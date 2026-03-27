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

class JoystickServo7Serial : public rclcpp::Node {
public:
  JoystickServo7Serial() : Node("joystick_servo7_serial_node") {
    // serial
    port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
    baud_ = declare_parameter<int>("baud", 115200);

    fd_ = open_serial(port_.c_str(), baud_);
    if (fd_ < 0) {
      RCLCPP_FATAL(get_logger(), "Failed to open serial port: %s", port_.c_str());
      throw std::runtime_error("Failed to open serial port");
    }

    // F710 axes
    axis_rx_ = declare_parameter<int>("axis_rx", 3); // right stick X
    axis_ry_ = declare_parameter<int>("axis_ry", 4); // right stick Y
    axis_lt_ = declare_parameter<int>("axis_lt", 2);
    axis_rt_ = declare_parameter<int>("axis_rt", 5);

    // buttons
    button_a_ = declare_parameter<int>("button_a", 0); // A -> propulsion macro
    button_b_ = declare_parameter<int>("button_b", 1); // B -> pouch trim (inflate-all)
    button_x_ = declare_parameter<int>("button_x", 2); // X -> pouch trim (deflate-all)

    //  update rate 
    update_rate_hz_ = declare_parameter<double>("update_rate_hz", 100.0);

    //  deadbands 
    deadband_stick_ = declare_parameter<double>("deadband_stick", 0.15);
    deadband_trig_  = declare_parameter<double>("deadband_trig", 0.05);

    //  servo limits 
    servo_min_   = declare_parameter<int>("servo_min", 0);
    servo_max_   = declare_parameter<int>("servo_max", 270);

    //  pouches (S1-S4) 
    default_pos_ = declare_parameter<int>("default_pos", 150);
    pouch_home_param_ = declare_parameter<int>("pouch_home", default_pos_);
    pouch_home_runtime_ = clamp(pouch_home_param_); // THIS will get updated by trim buttons

    // Stick deflection maps to +/- this many degrees away from home (for the active pouch)
    pouch_range_deg_ = declare_parameter<int>("pouch_range_deg", 150);

    // Slew-rate limit for pouch motion (deg/sec)
    pouch_max_speed_dps_ = declare_parameter<double>("pouch_max_speed_dps", 20.0);

    //  new: pouch trim behavior (X/B) 
    // How fast the shared home moves when holding X or B (deg/sec)
    pouch_trim_speed_dps_ = declare_parameter<double>("pouch_trim_speed_dps", 25.0);

    // Which direction is "deflate" for your mechanics:
    // -1 means deflate = decrease angle, +1 means deflate = increase angle
    pouch_deflate_sign_ = declare_parameter<int>("pouch_deflate_sign", -1);

    //  propulsion (S5) 
    propulsion_home_ = clamp(declare_parameter<int>("propulsion_home", default_pos_));

    //  triggers: full press threshold 
    trig_full_thresh_ = declare_parameter<double>("trig_full_thresh", 0.95);

    //  gripper (S6) via RT full press 
    gripper_home_     = clamp(declare_parameter<int>("gripper_home", default_pos_));
    gripper_ccw_pos_  = clamp(declare_parameter<int>("gripper_ccw_pos", default_pos_));
    gripper_speed_dps_= declare_parameter<double>("gripper_speed_dps", 100.0);

    //  stiffening (S7) via LT full press 
    stiff_home_      = clamp(declare_parameter<int>("stiff_home", default_pos_));
    stiff_cw_pos_    = clamp(declare_parameter<int>("stiff_cw_pos", default_pos_));
    stiff_speed_dps_ = declare_parameter<double>("stiff_speed_dps", 120.0);

    //  internal states 
    s1_ = s2_ = s3_ = s4_ = clamp(pouch_home_runtime_);
    s5_ = propulsion_home_;
    s6_ = gripper_home_;
    s7_ = stiff_home_;

    last_s1_=s1_; last_s2_=s2_; last_s3_=s3_; last_s4_=s4_;
    last_s5_=s5_; last_s6_=s6_; last_s7_=s7_;

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&JoystickServo7Serial::joyCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate_hz_),
      std::bind(&JoystickServo7Serial::tick, this));

    RCLCPP_INFO(get_logger(),
      "Servo node running.\n"
      "  Right stick (single-servo-per-direction): Up->S1, Down->S3, Right->S2, Left->S4\n"
      "  Hold X -> trim pouches (deflate all) | Hold B -> trim pouches (inflate all)\n"
      "  A -> propulsion macro (PROP)\n"
      "  RT full -> gripper | LT full -> stiffening\n");
  }

  ~JoystickServo7Serial() override {
    if (fd_ >= 0) ::close(fd_);
  }

private:
  //  Helpers 
  int clamp(int a) const { return std::clamp(a, servo_min_, servo_max_); }

  double axis(int idx) const {
    if (!last_joy_ || idx < 0 || idx >= (int)last_joy_->axes.size()) return 0.0;
    return last_joy_->axes[idx];
  }

  bool button(int idx) const {
    if (!last_joy_ || idx < 0 || idx >= (int)last_joy_->buttons.size()) return false;
    return last_joy_->buttons[idx] != 0;
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

  //  Serial 
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

  //  Main loop 
  void tick() {
    if (!last_joy_) return;

    const double dt = 1.0 / update_rate_hz_;

    //  A button edge: propulsion macro 
    const bool a_now = button(button_a_);
    if (a_now && !a_last_) {
      write_line("PROP\n");
    }
    a_last_ = a_now;

    // NEW: pouch trim by holding X/B 
    // Hold X: deflate all -> shift pouch_home_runtime_ continuously
    // Hold B: inflate all -> shift opposite direction continuously
    const bool x_now = button(button_x_);
    const bool b_now = button(button_b_);

    if (x_now || b_now) {
      // how many degrees per tick to shift the home
      double trim_step = pouch_trim_speed_dps_ * dt;
      int trim_step_i = std::max(1, (int)std::lround(trim_step));

      // deflate sign determines which direction is deflation on your mechanism
      // X = deflate, B = inflate (opposite)
      int dir = 0;
      if (x_now && !b_now) dir = pouch_deflate_sign_;
      if (b_now && !x_now) dir = -pouch_deflate_sign_;
      // if both pressed, do nothing

      if (dir != 0) {
        pouch_home_runtime_ = clamp(pouch_home_runtime_ + dir * trim_step_i);
      }
    }
    // On release: do nothing. The last pouch_home_runtime_ is now the new default.

    //  Right stick -> individual pouch targets around CURRENT home 
    const double rx = apply_deadband(axis(axis_rx_), deadband_stick_);
    const double ry = apply_deadband(axis(axis_ry_), deadband_stick_);

    int s1_target = pouch_home_runtime_;
    int s2_target = pouch_home_runtime_;
    int s3_target = pouch_home_runtime_;
    int s4_target = pouch_home_runtime_;

    // Single-servo-per-direction:
    // Up -> S1, Down -> S3, Right -> S2, Left -> S4
    if (ry > 0.0) {
      s1_target = clamp(pouch_home_runtime_ + (int)std::lround((double)pouch_range_deg_ * ry));
    } else if (ry < 0.0) {
      s3_target = clamp(pouch_home_runtime_ + (int)std::lround((double)pouch_range_deg_ * (-ry)));
    }

    if (rx > 0.0) {
      s2_target = clamp(pouch_home_runtime_ + (int)std::lround((double)pouch_range_deg_ * rx));
    } else if (rx < 0.0) {
      s4_target = clamp(pouch_home_runtime_ + (int)std::lround((double)pouch_range_deg_ * (-rx)));
    }

    //  Triggers -> gripper/stiff "full press" behavior 
    double rt01 = trigger01(axis(axis_rt_));
    double lt01 = trigger01(axis(axis_lt_));

    const bool rt_full = (rt01 >= trig_full_thresh_);
    const bool lt_full = (lt01 >= trig_full_thresh_);

    int s6_target = rt_full ? gripper_ccw_pos_ : gripper_home_;
    int s7_target = lt_full ? stiff_cw_pos_   : stiff_home_;

    //  Servo5 stays at home in SET stream (macro runs on Arduino) 
    int s5_target = propulsion_home_;

    //  Slew-rate limit for smooth motion 
    const double max_step_pouch = pouch_max_speed_dps_ * dt;
    s1_ = slew_to(s1_, s1_target, max_step_pouch);
    s2_ = slew_to(s2_, s2_target, max_step_pouch);
    s3_ = slew_to(s3_, s3_target, max_step_pouch);
    s4_ = slew_to(s4_, s4_target, max_step_pouch);

    const double max_step_grip = gripper_speed_dps_ * dt;
    s6_ = slew_to(s6_, s6_target, max_step_grip);

    const double max_step_stiff = stiff_speed_dps_ * dt;
    s7_ = slew_to(s7_, s7_target, max_step_stiff);

    // Keep s5 at propulsion_home (no slew needed)
    s5_ = s5_target;

    // Only send if changed
    if (s1_==last_s1_ && s2_==last_s2_ && s3_==last_s3_ &&
        s4_==last_s4_ && s5_==last_s5_ && s6_==last_s6_ && s7_==last_s7_) {
      return;
    }

    last_s1_=s1_; last_s2_=s2_; last_s3_=s3_; last_s4_=s4_;
    last_s5_=s5_; last_s6_=s6_; last_s7_=s7_;

    char buf[128];
    std::snprintf(buf, sizeof(buf),
      "SET:%d,%d,%d,%d,%d,%d,%d\n", s1_, s2_, s3_, s4_, s5_, s6_, s7_);
    write_line(buf);
  }

  //  members 
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Joy::SharedPtr last_joy_;

  std::string port_;
  int baud_{115200};
  int fd_{-1};

  int axis_rx_{3}, axis_ry_{4}, axis_lt_{2}, axis_rt_{5};

  int button_a_{0}, button_b_{1}, button_x_{2};
  bool a_last_{false};

  double update_rate_hz_{100.0};
  double deadband_stick_{0.15}, deadband_trig_{0.05};

  int servo_min_{0}, servo_max_{270};

  int default_pos_{150};

  // pouches
  int pouch_home_param_{150};
  int pouch_home_runtime_{150};
  int pouch_range_deg_{150};
  double pouch_max_speed_dps_{20.0};

  // pouch trim
  double pouch_trim_speed_dps_{25.0};
  int pouch_deflate_sign_{-1};

  // propulsion
  int propulsion_home_{150};

  // triggers
  double trig_full_thresh_{0.95};

  // gripper
  int gripper_home_{150};
  int gripper_ccw_pos_{150};
  double gripper_speed_dps_{100.0};

  // stiffening
  int stiff_home_{150};
  int stiff_cw_pos_{150};
  double stiff_speed_dps_{120.0};

  // servo states
  int s1_{150}, s2_{150}, s3_{150}, s4_{150}, s5_{150}, s6_{150}, s7_{150};
  int last_s1_{150}, last_s2_{150}, last_s3_{150}, last_s4_{150}, last_s5_{150}, last_s6_{150}, last_s7_{150};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickServo7Serial>());
  rclcpp::shutdown();
  return 0;
}
