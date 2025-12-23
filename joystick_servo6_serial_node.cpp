#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
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

    // ---- tuning ----
    update_rate_hz_ = declare_parameter<double>("update_rate_hz", 100.0);

    deadband_stick_ = declare_parameter<double>("deadband_stick", 0.15);
    deadband_trig_  = declare_parameter<double>("deadband_trig", 0.05);
    trig_threshold_ = declare_parameter<double>("trig_threshold", 0.10);

    pouch_step_deg_per_tick_ = declare_parameter<int>("pouch_step_deg_per_tick", 2);

    default_pos_ = declare_parameter<int>("default_pos", 150);
    servo_min_   = declare_parameter<int>("servo_min", 0);
    servo_max_   = declare_parameter<int>("servo_max", 270);

    prop_step_held_  = declare_parameter<int>("prop_step_held", 8);
    prop_step_relax_ = declare_parameter<int>("prop_step_relax", 6);

    grip_step_held_  = declare_parameter<int>("grip_step_held", 8);
    grip_step_relax_ = declare_parameter<int>("grip_step_relax", 6);

    // Initial targets
    s1_ = s2_ = s3_ = s4_ = s5_ = s6_ = clamp(default_pos_);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&JoystickServo6Serial::joyCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate_hz_),
      std::bind(&JoystickServo6Serial::tick, this));

    RCLCPP_INFO(get_logger(),
      "Servo node running | pouches=RX/RY | RT=propulsion | LT=gripper");
  }

  ~JoystickServo6Serial() override {
    if (fd_ >= 0) close(fd_);
  }

private:
  // ---------- helpers ----------
  int clamp(int a) const {
    return std::clamp(a, servo_min_, servo_max_);
  }

  double axis(int idx) const {
    if (!last_joy_ || idx < 0 || idx >= (int)last_joy_->axes.size()) return 0.0;
    return last_joy_->axes[idx];
  }

  static int sat(double v) {
    if (v >  0.5) return  1;
    if (v < -0.5) return -1;
    return 0;
  }

  double trigger01(double raw) const {
    // 1.0 unpressed → -1.0 pressed
    double p = (1.0 - raw) * 0.5;
    return std::clamp(p, 0.0, 1.0);
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    last_joy_ = msg;
  }

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

  // IMPORTANT: non-blocking-ish read
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "tcsetattr(%s) failed: %s", dev, std::strerror(errno));
    ::close(fd);
    return -1;
  }

  // Flush any garbage
  tcflush(fd, TCIOFLUSH);

  return fd;
  }


  // ---------- main loop ----------
  void tick() {
    if (!last_joy_) return;

    // ---- Right stick → pouches ----
    int hx = sat(axis(axis_rx_));
    int hy = sat(axis(axis_ry_));

    if (hx > 0) s1_ = clamp(s1_ + pouch_step_deg_per_tick_);
    if (hx < 0) s2_ = clamp(s2_ + pouch_step_deg_per_tick_);
    if (hy > 0) s3_ = clamp(s3_ + pouch_step_deg_per_tick_);
    if (hy < 0) s4_ = clamp(s4_ + pouch_step_deg_per_tick_);

    // ---- Triggers ----
    double rt = trigger01(axis(axis_rt_));
    double lt = trigger01(axis(axis_lt_));

    if (rt > trig_threshold_) {
      s5_ = clamp(s5_ + prop_step_held_);
    } else {
      s5_ += (s5_ > default_pos_) ? -prop_step_relax_ :
             (s5_ < default_pos_) ?  prop_step_relax_ : 0;
    }

    if (lt > trig_threshold_) {
      s6_ = clamp(s6_ + grip_step_held_);
    } else {
      s6_ += (s6_ > default_pos_) ? -grip_step_relax_ :
             (s6_ < default_pos_) ?  grip_step_relax_ : 0;
    }

    if (s1_==last_s1_ && s2_==last_s2_ && s3_==last_s3_ &&
        s4_==last_s4_ && s5_==last_s5_ && s6_==last_s6_)
      return;

    last_s1_=s1_; last_s2_=s2_; last_s3_=s3_;
    last_s4_=s4_; last_s5_=s5_; last_s6_=s6_;

    char buf[96];
    std::snprintf(buf, sizeof(buf),
      "SET:%d,%d,%d,%d,%d,%d\n", s1_,s2_,s3_,s4_,s5_,s6_);
    write_line(buf);
  }

  // ---------- members ----------
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Joy::SharedPtr last_joy_;

  std::string port_;
  int baud_{115200};
  int fd_{-1};

  int axis_rx_, axis_ry_, axis_lt_, axis_rt_;

  double update_rate_hz_;
  double deadband_stick_, deadband_trig_, trig_threshold_;

  int pouch_step_deg_per_tick_;
  int default_pos_, servo_min_, servo_max_;
  int prop_step_held_, prop_step_relax_;
  int grip_step_held_, grip_step_relax_;

  int s1_,s2_,s3_,s4_,s5_,s6_;
  int last_s1_{std::numeric_limits<int>::min()};
  int last_s2_{std::numeric_limits<int>::min()};
  int last_s3_{std::numeric_limits<int>::min()};
  int last_s4_{std::numeric_limits<int>::min()};
  int last_s5_{std::numeric_limits<int>::min()};
  int last_s6_{std::numeric_limits<int>::min()};

};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickServo6Serial>());
  rclcpp::shutdown();
  return 0;
}
