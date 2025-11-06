#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <sensor_msgs/msg/joy.hpp>

class LocalControllerNode : public rclcpp::Node {
public:
  LocalControllerNode() : Node("local_controller") {
    // Parameters let you adapt to different controller mappings
    // Typical Xbox on Linux with joy_linux: axes=[LX,LY,LT,RX,RY,RT,DPADX,DPADY], buttons=[A,B,X,Y,LB,RB,Back,Start,Guide,LStick,RStick]
    declare_parameter<int>("axis_lx", 0);
    declare_parameter<int>("axis_ly", 1);
    declare_parameter<int>("axis_rx", 3);
    declare_parameter<int>("axis_ry", 4);
    declare_parameter<int>("axis_dpad_x", 6);
    declare_parameter<int>("axis_dpad_y", 7);
    declare_parameter<int>("btn_a", 0);
    declare_parameter<int>("btn_b", 1);
    declare_parameter<int>("btn_x", 2);
    declare_parameter<int>("btn_y", 3);
    declare_parameter<int>("btn_lb", 4);
    declare_parameter<int>("btn_rb", 5);
    declare_parameter<int>("btn_back", 6);
    declare_parameter<int>("btn_start", 7);
    declare_parameter<double>("deadband", 0.05);      // small stick deadband
    declare_parameter<bool>("invert_ly", true);        // many prefer up=+1 => forward
    declare_parameter<bool>("invert_rx", false);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&LocalControllerNode::onJoy, this, std::placeholders::_1));

    cmd_pub_  = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    btn_pub_  = create_publisher<std_msgs::msg::Int16MultiArray>("/btn_array", 10);
    dpad_pub_ = create_publisher<std_msgs::msg::Int16MultiArray>("/dpad_array", 10);

    robot_mode_sub_ = create_subscription<std_msgs::msg::Int8>(
      "/robot_mode", 10, [this](std_msgs::msg::Int8::SharedPtr m){ robot_mode_ = m->data; });

    RCLCPP_INFO(get_logger(), "local_controller node started (listening to /joy)");
  }

private:
  // handy accessors for param lookups
  int P(const char* name){ return get_parameter(name).as_int(); }
  double Pd(const char* name){ return get_parameter(name).as_double(); }
  bool Pb(const char* name){ return get_parameter(name).as_bool(); }

  static double apply_deadband(double v, double db){
    return (std::abs(v) < db) ? 0.0 : v;
  }

  void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg){
    const auto &a = msg->axes;
    const auto &b = msg->buttons;

    // Fetch mapping
    int ax_lx = P("axis_lx"), ax_ly = P("axis_ly"), ax_rx = P("axis_rx");
    int ax_dpx= P("axis_dpad_x"), ax_dpy = P("axis_dpad_y");
    int iA=P("btn_a"), iB=P("btn_b"), iX=P("btn_x"), iY=P("btn_y");
    int iLB=P("btn_lb"), iRB=P("btn_rb"), iBack=P("btn_back"), iStart=P("btn_start");

    double db = Pd("deadband");
    bool inv_ly = Pb("invert_ly");
    bool inv_rx = Pb("invert_rx");

    auto get_axis = [&](int idx)->double{
      if (idx < 0 || idx >= (int)a.size()) return 0.0;
      return a[idx];
    };
    auto get_btn = [&](int idx)->int{
      if (idx < 0 || idx >= (int)b.size()) return 0;
      return b[idx];
    };

    // Read sticks
    double ly = apply_deadband(get_axis(ax_ly), db);
    double rx = apply_deadband(get_axis(ax_rx), db);
    if (inv_ly) ly = -ly;
    if (inv_rx) rx = -rx;

    // Bumpers gate behavior like your old code:
    bool LB = get_btn(iLB) != 0;
    bool RB = get_btn(iRB) != 0;

    geometry_msgs::msg::Twist twist{};
    std_msgs::msg::Int16MultiArray btns{};
    std_msgs::msg::Int16MultiArray dpad{};

    if (RB) {
      // drive mode
      twist.linear.x  = ly;   // forward/back
      twist.angular.z = rx;   // yaw
    }

    if (LB) {
      // First 6 buttons as before: {A, X, Y, B, Start, Back}
      btns.data = {
        (int16_t)get_btn(iA),
        (int16_t)get_btn(iX),
        (int16_t)get_btn(iY),
        (int16_t)get_btn(iB),
        (int16_t)get_btn(iStart),
        (int16_t)get_btn(iBack)
      };

      // D-pad (convert axes to discrete L,R,U,D)
      double dpx = get_axis(ax_dpx), dpy = get_axis(ax_dpy);
      dpad.data = {
        (int16_t)(dpx < -0.5),   // Left
        (int16_t)(dpx >  0.5),   // Right
        (int16_t)(dpy >  0.5),   // Up
        (int16_t)(dpy < -0.5)    // Down
      };
    }

    if (robot_mode_ == 0) {
      cmd_pub_->publish(twist);
      dpad_pub_->publish(dpad);
    }
    btn_pub_->publish(btns);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "cmd: v=%.2f w=%.2f (mode=%d)", twist.linear.x, twist.angular.z, robot_mode_);
  }

  int robot_mode_{0};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr robot_mode_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr btn_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr dpad_pub_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalControllerNode>());
  rclcpp::shutdown();
  return 0;
}