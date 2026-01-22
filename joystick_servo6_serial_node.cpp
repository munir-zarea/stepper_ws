#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

class JoystickServo6Serial : public rclcpp::Node {
public:
  JoystickServo6Serial() : Node("joystick_servo6_serial_node") {
    // ---- serial ----
    port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
    baud_ = declare_parameter<int>("baud", 115200);

    fd_ = open_serial(port_.c_str(), baud_);
    if (fd_ < 0) {
      RCLCPP_FATAL(get_logger(), "Failed to open serial port: %s", port_.c_str());
      throw std::runtime_error("Failed to open serial port");
    }

    // ---- joystick mapping ----
    axis_rx_ = declare_parameter<int>("axis_rx", 3); // right stick X
    axis_ry_ = declare_parameter<int>("axis_ry", 4); // right stick Y
    axis_lt_ = declare_parameter<int>("axis_lt", 2);
    axis_rt_ = declare_parameter<int>("axis_rt", 5);

    // NEW: button hold for servo7 (set to whatever button index you want)
    // Common Xbox mapping: RB=5, LB=4, A=0, B=1, X=2, Y=3, Start=7, Back=6 (varies by driver)
    button_servo7_hold_ = declare_parameter<int>("button_servo7_hold", 0); // default RB
    servo7_bump_deg_    = declare_parameter<int>("servo7_bump_deg", 15);   // "short distance" CW
    servo7_max_speed_dps_ = declare_parameter<double>("servo7_max_speed_dps", 90.0);

    // ---- update rate ----
    update_rate_hz_ = declare_parameter<double>("update_rate_hz", 60.0);

    // ---- deadbands ----
    deadband_stick_ = declare_parameter<double>("deadband_stick", 0.15);
    deadband_trig_  = declare_parameter<double>("deadband_trig", 0.05);

    // ---- servo limits / homes ----
    servo_min_   = declare_parameter<int>("servo_min", 0);
    servo_max_   = declare_parameter<int>("servo_max", 270);

    default_pos_ = declare_parameter<int>("default_pos", 150);
    pouch_home_  = declare_parameter<int>("pouch_home", default_pos_);

    // Stick deflection maps to +/- this many degrees away from home.
    pouch_range_deg_ = declare_parameter<int>("pouch_range_deg", 150);

    // Smoothing (max degrees per second)
    pouch_max_speed_dps_ = declare_parameter<double>("pouch_max_speed_dps", 20.0);

    // Triggers absolute mapping (servos 5 and 6 independent)
    trig_range_deg_ = declare_parameter<int>("trig_range_deg", 100);
    trig_max_speed_dps_ = declare_parameter<double>("trig_max_speed_dps", 60.0);

    // Initial positions
    s1_ = s2_ = s3_ = s4_ = s5_ = s6_ = s7_ = clamp(default_pos_);
    last_s1_=s1_; last_s2_=s2_; last_s3_=s3_;
    last_s4_=s4_; last_s5_=s5_; last_s6_=s6_; last_s7_=s7_;

    // Servo7 base is "whatever it was before you held the button"
    servo7_base_ = s7_;

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&JoystickServo6Serial::joyCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate_hz_),
      std::bind(&JoystickServo6Serial::tick, this));

    RCLCPP_INFO(get_logger(),
      "Servo node running | Up->S1, Down->S3, Right->S2, Left->S4 | RT=S5 LT=S6 | HoldBtn->S7 bump");
  }

  ~JoystickServo6Serial() override {
    if (fd_ >= 0) ::close(fd_);
  }

private:
  // ---------- helpers ----------
  int clamp(int a) const { return std::clamp(a, servo_min_, servo_max_); }

  double axis(int idx) const {
    if (!last_joy_ || idx < 0 || idx >= (int)last_joy_->axes.size()) return 0.0;
    return last_joy_->axes[idx];
  }

  int button(int idx) const {
    if (!last_joy_ || idx < 0 || idx >= (int)last_joy_->buttons.size()) return 0;
    return last_joy_->buttons[idx];
  }

  double apply_deadband(double v, double db) const {
    if (std::fabs(v) < db) return 0.0;
    double sign = (v >= 0.0) ? 1.0 : -1.0;
    double mag  = (std::fabs(v) - db) / (1.0 - db);
    return sign * std::clamp(mag, 0.0, 1.0);
  }

  int slew_to(int cur, int target, double max_step_deg) const {
    const int step = std::max(1, (int)std::ceil(max_step_deg));
    if (cur < target) return std::min(target, cur + step);
    if (cur > target) return std::max(target, cur - step);
    return cur;
  }

  double trigger01(double raw) const {
    // Assumes: 1.0 unpressed → -1.0 pressed (common)
    double p = (1.0 - raw) * 0.5;
    p = std::clamp(p, 0.0, 1.0);
    if (p < deadband_trig_) p = 0.0;
    return p;
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) { last_joy_ = msg; }

  void write_line(const std::string& s) {
    if (fd_ >= 0) ::write(fd_, s.c_str(), s.size());
  }

  // ---------- serial ----------
  static speed_t baud_to_flag(int baud) {
    return baud == 9600 ? B9600 : baud == 57600 ? B57600 : B115200;
  }

  int open_serial(const char* dev, int baud) {
    int fd = ::open(dev, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
      RCLCPP_ERROR(get_logger(), "open(%s) failed: %s", dev, std::strerror(errno));
      return -1;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr(%s) failed: %s", dev, std::strerror(errno));
      ::close(fd);
      return -1;
    }

    cfmakeraw(&tty);

    speed_t spd = baud_to_flag(baud);
    if (cfsetispeed(&tty, spd) != 0 || cfsetospeed(&tty, spd) != 0) {
      RCLCPP_ERROR(get_logger(), "cfsetispeed/cfsetospeed failed: %s", std::strerror(errno));
      ::close(fd);
      return -1;
    }

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr(%s) failed: %s", dev, std::strerror(errno));
      ::close(fd);
      return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
  }

  // ---------- main loop ----------
  void tick() {
    if (!last_joy_) return;

    const double dt = 1.0 / update_rate_hz_;

    // Read stick as absolute position (-1..1), apply deadband
    const double rx = apply_deadband(axis(axis_rx_), deadband_stick_);
    const double ry = apply_deadband(axis(axis_ry_), deadband_stick_);

    // Default all pouch targets to home
    int s1_target = pouch_home_;
    int s2_target = pouch_home_;
    int s3_target = pouch_home_;
    int s4_target = pouch_home_;

    // Single-servo-per-direction:
    // Up -> S1, Down -> S3, Right -> S2, Left -> S4
    if (ry > 0.0) {
      s1_target = clamp(pouch_home_ + (int)std::lround((double)pouch_range_deg_ * ry));
    } else if (ry < 0.0) {
      s3_target = clamp(pouch_home_ + (int)std::lround((double)pouch_range_deg_ * (-ry)));
    }

    if (rx > 0.0) {
      s2_target = clamp(pouch_home_ + (int)std::lround((double)pouch_range_deg_ * rx));
    } else if (rx < 0.0) {
      s4_target = clamp(pouch_home_ + (int)std::lround((double)pouch_range_deg_ * (-rx)));
    }

    // Triggers base targets (5 and 6 independent)
    double rt01 = trigger01(axis(axis_rt_));
    double lt01 = trigger01(axis(axis_lt_));

    int s5_target = clamp(default_pos_ + (int)std::lround((double)trig_range_deg_ * rt01));
    int s6_target = clamp(default_pos_ + (int)std::lround((double)trig_range_deg_ * lt01));

    // ---- Servo7 hold-button bump behavior ----
    const bool held = (button(button_servo7_hold_) != 0);

    // Edge detect to capture "normal position before press"
    if (held && !servo7_held_last_) {
      // on press: remember where we were
      servo7_base_ = s7_;
    }
    servo7_held_last_ = held;

    int s7_target = servo7_base_;
    if (held) {
      // CW bump = +deg (if your servo direction is opposite, make this negative)
      s7_target = clamp(servo7_base_ + servo7_bump_deg_);
    }

    // Slew-rate limit for smooth motion
    const double max_step_pouch = pouch_max_speed_dps_ * dt;
    s1_ = slew_to(s1_, s1_target, max_step_pouch);
    s2_ = slew_to(s2_, s2_target, max_step_pouch);
    s3_ = slew_to(s3_, s3_target, max_step_pouch);
    s4_ = slew_to(s4_, s4_target, max_step_pouch);

    const double max_step_trig = trig_max_speed_dps_ * dt;
    s5_ = slew_to(s5_, s5_target, max_step_trig);
    s6_ = slew_to(s6_, s6_target, max_step_trig);

    const double max_step_s7 = servo7_max_speed_dps_ * dt;
    s7_ = slew_to(s7_, s7_target, max_step_s7);

    // Only send if something changed
    if (s1_==last_s1_ && s2_==last_s2_ && s3_==last_s3_ &&
        s4_==last_s4_ && s5_==last_s5_ && s6_==last_s6_ && s7_==last_s7_) {
      return;
    }

    last_s1_=s1_; last_s2_=s2_; last_s3_=s3_;
    last_s4_=s4_; last_s5_=s5_; last_s6_=s6_; last_s7_=s7_;

    char buf[128];
    std::snprintf(buf, sizeof(buf),
      "SET:%d,%d,%d,%d,%d,%d,%d\n", s1_, s2_, s3_, s4_, s5_, s6_, s7_);
    write_line(buf);
  }

  // ---------- members ----------
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Joy::SharedPtr last_joy_;

  std::string port_;
  int baud_{115200};
  int fd_{-1};

  int axis_rx_{3}, axis_ry_{4}, axis_lt_{2}, axis_rt_{5};

  // NEW servo7 button bump params/state
  int button_servo7_hold_{5};
  int servo7_bump_deg_{15};
  double servo7_max_speed_dps_{90.0};
  bool servo7_held_last_{false};
  int servo7_base_{150};

  double update_rate_hz_{60.0};
  double deadband_stick_{0.15}, deadband_trig_{0.05};

  int servo_min_{0}, servo_max_{270};
  int default_pos_{150};
  int pouch_home_{150};

  int pouch_range_deg_{60};
  double pouch_max_speed_dps_{20.0};

  int trig_range_deg_{100};
  double trig_max_speed_dps_{60.0};

  int s1_{150}, s2_{150}, s3_{150}, s4_{150}, s5_{150}, s6_{150}, s7_{150};
  int last_s1_{150}, last_s2_{150}, last_s3_{150}, last_s4_{150}, last_s5_{150}, last_s6_{150}, last_s7_{150};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickServo6Serial>());
  rclcpp::shutdown();
  return 0;
}
