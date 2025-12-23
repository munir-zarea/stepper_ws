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
    // ---- joystick mapping (alpha/theta) ----
    axis_x_   = declare_parameter<int>("axis_x", 0);     // left stick X -> theta
    axis_y_   = declare_parameter<int>("axis_y", 1);     // left stick Y -> alpha
    deadband_ = declare_parameter<double>("deadband", 0.15);

    // ---- Stepper4 mapping (Option 1: buttons only, NO triggers) ----
    // Typical Xbox mapping: LB=4, RB=5
    btn_step4_neg_ = declare_parameter<int>("btn_step4_neg", 4); // LB -> negative
    btn_step4_pos_ = declare_parameter<int>("btn_step4_pos", 5); // RB -> positive

    // ---- motion tuning ----
    movespeed_deg_per_tick_ = declare_parameter<double>("movespeed_deg_per_tick", 0.3);
    update_rate_hz_         = declare_parameter<double>("update_rate_hz", 60.0);

    // Stepper4: how many steps to add per tick while button held (slows/faster feel)
    step4_steps_per_tick_ = declare_parameter<double>("step4_steps_per_tick", 5.0);

    // ---- model constants (match your kinematics figure) ----
    // d = tendon offset radius, r = spool radius
    d_ = declare_parameter<double>("d", 0.9);
    r_ = declare_parameter<double>("r", 2.1);

    // Alignment knobs (use if directions/signs are flipped)
    theta_offset_deg_ = declare_parameter<double>("theta_offset_deg", 0.0);
    invert_phi_       = declare_parameter<bool>("invert_phi", false);

    // ---- steps conversion ----
    steps_per_rev_effective_ = declare_parameter<int>("steps_per_rev_effective", 3200);

    // ---- initial conditions ----
    alpha_deg_ = declare_parameter<double>("alpha_start_deg", 0.0);
    theta_deg_ = declare_parameter<double>("theta_start_deg", 270.0);

    // ---- clamps ----
    alpha_min_ = declare_parameter<double>("alpha_min_deg", 0.0);
    alpha_max_ = declare_parameter<double>("alpha_max_deg", 260.0);

    // ---- Stepper4 travel limits (NEW: cap distance from home) ----
    step4_home_steps_        = declare_parameter<long>("step4_home_steps", 0);
    step4_max_travel_steps_  = declare_parameter<long>("step4_max_travel_steps", 4000);

    // Compute hard limits + initialize stepper4 state
    step4_steps_     = step4_home_steps_;
    step4_min_steps_ = step4_home_steps_ - step4_max_travel_steps_;
    step4_max_steps_ = step4_home_steps_ + step4_max_travel_steps_;

    // ---- serial ----
    port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
    baud_ = declare_parameter<int>("baud", 115200);

    fd_ = open_serial(port_.c_str(), baud_);
    if (fd_ < 0) {
      RCLCPP_FATAL(get_logger(), "Failed to open serial port: %s", port_.c_str());
      throw std::runtime_error("Failed to open serial port");
    }

    // Enable drivers on Arduino
    write_line("EN 1\n");

    // Subscribe to joy
    sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&StepperJoy::on_joy, this, std::placeholders::_1));

    // Fixed-rate tick
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate_hz_),
      std::bind(&StepperJoy::tick, this)
    );

    RCLCPP_INFO(get_logger(),
      "stepper_joy_node started\n"
      "  port=%s baud=%d\n"
      "  axes: x=%d y=%d deadband=%.2f\n"
      "  alpha/theta step=%.3f deg/tick rate=%.1f Hz\n"
      "  model: d=%.4f r=%.4f theta_offset=%.2fdeg invert_phi=%s\n"
      "  steps_per_rev_effective=%d\n"
      "  stepper4 buttons: pos=%d neg=%d | step=%.1f steps/tick\n"
      "  stepper4 home=%ld max_travel=%ld => clamp=[%ld..%ld]\n"
      "  Serial command: P t1 t2 t3 t4",
      port_.c_str(), baud_,
      axis_x_, axis_y_, deadband_,
      movespeed_deg_per_tick_, update_rate_hz_,
      d_, r_, theta_offset_deg_, invert_phi_ ? "true" : "false",
      steps_per_rev_effective_,
      btn_step4_pos_, btn_step4_neg_, step4_steps_per_tick_,
      step4_home_steps_, step4_max_travel_steps_, step4_min_steps_, step4_max_steps_);
  }

  ~StepperJoy() override {
    if (fd_ >= 0) close(fd_);
  }

private:
  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg) { last_joy_ = msg; }

  static double wrap_0_360(double deg) {
    while (deg >= 360.0) deg -= 360.0;
    while (deg < 0.0) deg += 360.0;
    return deg;
  }

  long clamp_steps(long v, long lo, long hi) const {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  }

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

  // Kinematics: (alpha,theta) -> phi revolutions
  void alphaThetaToPhiRev(double alpha_deg, double theta_deg, double phi_rev[3]) const {
    const double alpha_rad = alpha_deg * M_PI / 180.0;
    const double theta_rad = (theta_deg + theta_offset_deg_) * M_PI / 180.0;

    const double scale = (alpha_rad * d_ / r_);

    double phi1 = scale * std::cos(theta_rad);
    double phi2 = scale * std::cos(theta_rad + 2.0 * M_PI / 3.0);
    double phi3 = scale * std::cos(theta_rad + 4.0 * M_PI / 3.0);

    if (invert_phi_) { phi1 = -phi1; phi2 = -phi2; phi3 = -phi3; }

    phi_rev[0] = phi1 / (2.0 * M_PI);
    phi_rev[1] = phi2 / (2.0 * M_PI);
    phi_rev[2] = phi3 / (2.0 * M_PI);
  }

  void tick() {
    if (!last_joy_) return;

    // ---- alpha/theta from joystick ----
    double x = get_axis(axis_x_);
    double y = get_axis(axis_y_);

    if (std::abs(x) < deadband_) x = 0.0;
    if (std::abs(y) < deadband_) y = 0.0;

    // Held stick => constant increments
    if (x > 0.0) theta_deg_ += movespeed_deg_per_tick_;
    else if (x < 0.0) theta_deg_ -= movespeed_deg_per_tick_;

    if (y > 0.0) alpha_deg_ += movespeed_deg_per_tick_;
    else if (y < 0.0) alpha_deg_ -= movespeed_deg_per_tick_;

    // Clamp/wrap
    alpha_deg_ = std::clamp(alpha_deg_, alpha_min_, alpha_max_);
    theta_deg_ = wrap_0_360(theta_deg_);

    // ---- Kinematics -> steps for motors 1-3 ----
    double phi_rev[3];
    alphaThetaToPhiRev(alpha_deg_, theta_deg_, phi_rev);

    const long t1 = lround(phi_rev[0] * (double)steps_per_rev_effective_);
    const long t2 = lround(phi_rev[1] * (double)steps_per_rev_effective_);
    const long t3 = lround(phi_rev[2] * (double)steps_per_rev_effective_);

    // ---- Stepper4 via buttons (RB/LB) with capped travel ----
    const bool pos = get_button(btn_step4_pos_); // RB
    const bool neg = get_button(btn_step4_neg_); // LB

    long delta4_steps = 0;
    if (pos && !neg) delta4_steps =  (long)lround(step4_steps_per_tick_);
    if (neg && !pos) delta4_steps = -(long)lround(step4_steps_per_tick_);

    if (delta4_steps != 0) {
      step4_steps_ = clamp_steps(step4_steps_ + delta4_steps, step4_min_steps_, step4_max_steps_);
    }

    // ---- Send only if changed ----
    if (t1 == last_t1_ && t2 == last_t2_ && t3 == last_t3_ && step4_steps_ == last_t4_) return;

    last_t1_ = t1; last_t2_ = t2; last_t3_ = t3; last_t4_ = step4_steps_;

    char buf[140];
    std::snprintf(buf, sizeof(buf), "P %ld %ld %ld %ld\n", t1, t2, t3, step4_steps_);
    write_line(buf);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "alpha=%.2fdeg theta=%.2fdeg -> steps [%ld %ld %ld] | t4=%ld | LB=%d RB=%d",
      alpha_deg_, theta_deg_, t1, t2, t3, step4_steps_,
      (int)neg, (int)pos);
  }

  // ---------- serial helpers ----------
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

  void write_line(const std::string& s) {
    if (fd_ < 0) return;
    ::write(fd_, s.c_str(), s.size());
  }

  // ---------- members ----------
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Joy::SharedPtr last_joy_;

  // joystick
  int axis_x_{0}, axis_y_{1};
  double deadband_{0.15};

  // stepper4 buttons
  int btn_step4_neg_{4}; // LB
  int btn_step4_pos_{5}; // RB

  // tuning
  double movespeed_deg_per_tick_{0.3};
  double update_rate_hz_{60.0};
  double step4_steps_per_tick_{5.0};

  // model params
  double d_{0.9};
  double r_{2.1};
  double theta_offset_deg_{0.0};
  bool   invert_phi_{false};

  int steps_per_rev_effective_{3200};

  // state
  double alpha_deg_{0.0}, theta_deg_{270.0};
  double alpha_min_{0.0}, alpha_max_{260.0};

  // Stepper4 state + limits
  long step4_steps_{0};
  long step4_home_steps_{0};
  long step4_max_travel_steps_{4000};
  long step4_min_steps_{0};
  long step4_max_steps_{0};

  // last sent targets
  long last_t1_{std::numeric_limits<long>::min()};
  long last_t2_{std::numeric_limits<long>::min()};
  long last_t3_{std::numeric_limits<long>::min()};
  long last_t4_{std::numeric_limits<long>::min()};

  // serial
  std::string port_{"/dev/ttyACM0"};
  int baud_{115200};
  int fd_{-1};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StepperJoy>());
  rclcpp::shutdown();
  return 0;
}
