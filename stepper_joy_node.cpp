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

class StepperJoy : public rclcpp::Node {
public:
  StepperJoy() : Node("stepper_joy_node") {
    //  Joystick 
    axis_x_   = declare_parameter<int>("axis_x", 0);
    deadband_ = declare_parameter<double>("deadband", 0.20);
    invert_x_ = declare_parameter<bool>("invert_x", false);

    // Buttons
    btn_home_    = declare_parameter<int>("btn_home", 3);   // Y
    use_deadman_ = declare_parameter<bool>("use_deadman", false);
    btn_deadman_ = declare_parameter<int>("btn_deadman", 0); // A

    //  Update rate 
    update_rate_hz_ = declare_parameter<double>("update_rate_hz", 100.0);

    //  Active stepper (1..3) 
    joy_stepper_id_     = declare_parameter<int>("joy_stepper_id", 2); // you want 2
    joy_steps_per_tick_ = declare_parameter<long>("joy_steps_per_tick", 5);

    // Start/home (1..3)
    t1_ = declare_parameter<long>("t1_start_steps", 0);
    t2_ = declare_parameter<long>("t2_start_steps", 0);
    t3_ = declare_parameter<long>("t3_start_steps", 0);

    t1_home_ = declare_parameter<long>("t1_home_steps", t1_);
    t2_home_ = declare_parameter<long>("t2_home_steps", t2_);
    t3_home_ = declare_parameter<long>("t3_home_steps", t3_);

    // Origin for extra-slack threshold (default start)
    t2_origin_ = declare_parameter<long>("t2_origin_steps", t2_);

    //  Slack behavior on other two steppers 
    slack_fraction_base_  = declare_parameter<double>("slack_fraction_base", 0.05);
    slack_min_steps_base_ = declare_parameter<long>("slack_min_steps_base", 0);

    slack_extra_enable_ = declare_parameter<bool>("slack_extra_enable", true);
    slack_extra_threshold_steps_ = declare_parameter<long>("slack_extra_threshold_steps", 200);
    slack_fraction_extra_  = declare_parameter<double>("slack_fraction_extra", 0.10);
    slack_min_steps_extra_ = declare_parameter<long>("slack_min_steps_extra", 2);
    slack_extra_cap_steps_ = declare_parameter<long>("slack_extra_cap_steps", 50);

    //  Stepper4 (RB/LB) 
    send_step4_ = declare_parameter<bool>("send_step4", true);

    btn_step4_neg_ = declare_parameter<int>("btn_step4_neg", 4); // LB
    btn_step4_pos_ = declare_parameter<int>("btn_step4_pos", 5); // RB
    step4_steps_per_tick_ = declare_parameter<double>("step4_steps_per_tick", 5.0);

    step4_home_steps_       = declare_parameter<long>("step4_home_steps", 0);
    step4_max_travel_steps_ = declare_parameter<long>("step4_max_travel_steps", 2000);

    step4_steps_     = step4_home_steps_;
    step4_min_steps_ = step4_home_steps_ - step4_max_travel_steps_;
    step4_max_steps_ = step4_home_steps_ + step4_max_travel_steps_;

    // If true, HOME also homes stepper4
    home_step4_too_ = declare_parameter<bool>("home_step4_too", false);

    //  Serial 
    port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
    baud_ = declare_parameter<int>("baud", 115200);

    send_enable_cmd_ = declare_parameter<bool>("send_enable_cmd", false);
    enable_cmd_      = declare_parameter<std::string>("enable_cmd", "EN 1\n");

    // P includes t4
    cmd_p_format_     = declare_parameter<std::string>("cmd_p_format", "P %ld %ld %ld %ld\n");

    // Optional Z for stepper4
    send_step4_z_     = declare_parameter<bool>("send_step4_z", true);
    cmd_step4_format_ = declare_parameter<std::string>("cmd_step4_format", "Z %ld\n");

    fd_ = open_serial(port_.c_str(), baud_);
    if (fd_ < 0) {
      RCLCPP_FATAL(get_logger(), "Failed to open serial port: %s", port_.c_str());
      throw std::runtime_error("Failed to open serial port");
    }
    if (send_enable_cmd_) write_line(enable_cmd_);

    send_p_if_changed(true);
    if (send_step4_ && send_step4_z_) send_step4_if_changed(true);

    sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&StepperJoy::on_joy, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate_hz_),
      std::bind(&StepperJoy::tick, this)
    );

    RCLCPP_INFO(get_logger(),
      "stepper_joy_node started\n"
      "  Active stepper=%d (1..3), steps/tick=%ld\n"
      "  Stepper4: RB=%d (+) LB=%d (-) steps/tick=%.1f travel=±%ld (home=%ld)\n"
      "  serial: %s @ %d\n",
      joy_stepper_id_, joy_steps_per_tick_,
      btn_step4_pos_, btn_step4_neg_, step4_steps_per_tick_,
      step4_max_travel_steps_, step4_home_steps_,
      port_.c_str(), baud_);
  }

  ~StepperJoy() override {
    if (fd_ >= 0) ::close(fd_);
  }

private:
  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg) { last_joy_ = msg; }

  double get_axis(int idx) const {
    if (!last_joy_) return 0.0;
    if (idx < 0 || idx >= (int)last_joy_->axes.size()) return 0.0;
    return last_joy_->axes[idx];
  }

  bool get_button(int idx) const {
    if (!last_joy_) return false;
    if (idx < 0 || idx >= (int)last_joy_->buttons.size()) return false;
    return last_joy_->buttons[idx] != 0;
  }

  static long clamp_steps(long v, long lo, long hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  }

  void tick() {
    if (!last_joy_) return;
    if (use_deadman_ && !get_button(btn_deadman_)) return;

    const int sid = std::clamp(joy_stepper_id_, 1, 3);

    // HOME
    if (get_button(btn_home_)) {
      t1_ = t1_home_;
      t2_ = t2_home_;
      t3_ = t3_home_;

      // reset origin to home (optional but usually desirable)
      t2_origin_ = t2_home_;

      if (home_step4_too_) step4_steps_ = step4_home_steps_;

      send_p_if_changed(true);
      if (send_step4_ && send_step4_z_) send_step4_if_changed(true);
      return;
    }

    // - Stepper 1..3 joystick logic -
    double x = get_axis(axis_x_);
    if (invert_x_) x = -x;

    long delta = 0;
    if (x > deadband_) delta =  joy_steps_per_tick_;
    if (x < -deadband_) delta = -joy_steps_per_tick_;

    bool p_changed = false;
    if (delta != 0) {
      // main move
      if (sid == 1) t1_ += delta;
      if (sid == 2) t2_ += delta;
      if (sid == 3) t3_ += delta;

      // baseline slack on other two
      long base_slack = (long)std::llround(std::fabs((double)delta) * std::clamp(slack_fraction_base_, 0.0, 1.0));
      if (base_slack < slack_min_steps_base_) base_slack = slack_min_steps_base_;

      // extra slack when moving RIGHT and t2 beyond threshold
      long extra_slack = 0;
      if (slack_extra_enable_ && delta > 0) {
        const long t2_disp = t2_ - t2_origin_;
        if (t2_disp >= slack_extra_threshold_steps_) {
          extra_slack = (long)std::llround(std::fabs((double)delta) * std::clamp(slack_fraction_extra_, 0.0, 1.0));
          if (extra_slack < slack_min_steps_extra_) extra_slack = slack_min_steps_extra_;
          extra_slack = std::min(extra_slack, slack_extra_cap_steps_);
        }
      }

      const long relief = (delta > 0) ? -(base_slack + extra_slack) : +(base_slack);

      if (sid == 1) { t2_ += relief; t3_ += relief; }
      if (sid == 2) { t1_ += relief; t3_ += relief; }
      if (sid == 3) { t1_ += relief; t2_ += relief; }

      p_changed = true;
    }

    // - Stepper 4 RB/LB logic -
    bool step4_changed = false;
    if (send_step4_) {
      const bool pos = get_button(btn_step4_pos_);
      const bool neg = get_button(btn_step4_neg_);
      long delta4 = 0;

      if (pos && !neg) delta4 =  (long)std::lround(step4_steps_per_tick_);
      if (neg && !pos) delta4 = -(long)std::lround(step4_steps_per_tick_);

      if (delta4 != 0) {
        step4_steps_ = clamp_steps(step4_steps_ + delta4, step4_min_steps_, step4_max_steps_);
        step4_changed = true;
      }
    }

    if (p_changed || step4_changed) {
      send_p_if_changed(false);
      if (send_step4_ && send_step4_z_ && step4_changed) {
        send_step4_if_changed(false);
      }
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "sid=%d x=%.2f | t1=%ld t2=%ld t3=%ld | t2_disp=%ld | step4=%ld",
      sid, x, t1_, t2_, t3_, (t2_ - t2_origin_), step4_steps_);
  }

  void send_p_if_changed(bool force) {
    const long t4 = step4_steps_;

    if (!force &&
        t1_ == last_sent_t1_ &&
        t2_ == last_sent_t2_ &&
        t3_ == last_sent_t3_ &&
        t4 == last_sent_t4_) {
      return;
    }

    last_sent_t1_ = t1_;
    last_sent_t2_ = t2_;
    last_sent_t3_ = t3_;
    last_sent_t4_ = t4;

    char buf[180];
    std::snprintf(buf, sizeof(buf), cmd_p_format_.c_str(), t1_, t2_, t3_, t4);
    write_line(buf);
  }

  void send_step4_if_changed(bool force) {
    if (!force && step4_steps_ == last_sent_step4_) return;
    last_sent_step4_ = step4_steps_;

    char buf[120];
    std::snprintf(buf, sizeof(buf), cmd_step4_format_.c_str(), step4_steps_);
    write_line(buf);
  }

  //  serial 
  static speed_t baud_to_flag(int baud) {
    switch (baud) {
      case 9600: return B9600;
      case 57600: return B57600;
      case 115200: return B115200;
      default: return B115200;
    }
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
      RCLCPP_ERROR(get_logger(), "cfsetispeed/cfsetospeed failed on %s: %s", dev, std::strerror(errno));
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

  void write_line(const std::string& s) {
    if (fd_ < 0) return;
    (void)::write(fd_, s.c_str(), s.size());
  }

  //  members 
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Joy::SharedPtr last_joy_;

  int axis_x_{0};
  double deadband_{0.20};
  bool invert_x_{false};

  int btn_home_{3};
  bool use_deadman_{false};
  int btn_deadman_{0};

  double update_rate_hz_{100.0};

  int joy_stepper_id_{2};
  long joy_steps_per_tick_{5};

  long t1_{0}, t2_{0}, t3_{0};
  long t1_home_{0}, t2_home_{0}, t3_home_{0};
  long t2_origin_{0};

  double slack_fraction_base_{0.05};
  long slack_min_steps_base_{0};

  bool slack_extra_enable_{true};
  long slack_extra_threshold_steps_{200};
  double slack_fraction_extra_{0.10};
  long slack_min_steps_extra_{2};
  long slack_extra_cap_steps_{50};

  // stepper4
  bool send_step4_{true};
  bool send_step4_z_{true};
  int btn_step4_neg_{4}, btn_step4_pos_{5};
  double step4_steps_per_tick_{5.0};
  long step4_steps_{0};
  long step4_home_steps_{0};
  long step4_max_travel_steps_{2000};
  long step4_min_steps_{0};
  long step4_max_steps_{0};
  bool home_step4_too_{false};

  long last_sent_t1_{std::numeric_limits<long>::min()};
  long last_sent_t2_{std::numeric_limits<long>::min()};
  long last_sent_t3_{std::numeric_limits<long>::min()};
  long last_sent_t4_{std::numeric_limits<long>::min()};
  long last_sent_step4_{std::numeric_limits<long>::min()};

  std::string port_{"/dev/ttyACM0"};
  int baud_{115200};
  int fd_{-1};

  bool send_enable_cmd_{false};
  std::string enable_cmd_{"EN 1\n"};

  std::string cmd_p_format_{"P %ld %ld %ld %ld\n"};
  std::string cmd_step4_format_{"Z %ld\n"};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StepperJoy>());
  rclcpp::shutdown();
  return 0;
}
