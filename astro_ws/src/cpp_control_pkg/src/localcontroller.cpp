#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>

using namespace std::chrono_literals;

class XboxController {
public:
    XboxController() {
        // Initialize all values
        LeftJoystickY = 0.0;
        LeftJoystickX = 0.0;
        RightJoystickY = 0.0;
        RightJoystickX = 0.0;
        A = X = Y = B = LeftThumb = RightThumb = Back = Start = Left = Right = Up = Down = 0;
        RightBumper = LeftBumper = 0;

        monitor_thread = std::thread(&XboxController::monitorController, this);
        monitor_thread.detach();
    }

    double readLeftStickX() const { return -LeftJoystickX; }
    double readLeftStickY() const { return -LeftJoystickY; }
    double readRightStickX() const { return -RightJoystickX; }
    double readRightStickY() const { return -RightJoystickY; }

    std::vector<int> readBtn() const {
        return {A, X, Y, B, Start, Back, Up, Down, Left, Right, LeftThumb, RightThumb};
    }

    std::string readBumper() const {
        if (RightBumper) return "RB";
        if (LeftBumper) return "LB";
        return "";
    }

private:
    double LeftJoystickY, LeftJoystickX, RightJoystickY, RightJoystickX;
    int A, X, Y, B, LeftThumb, RightThumb, Back, Start, Left, Right, Up, Down;
    int RightBumper, LeftBumper;

    std::thread monitor_thread;

    void monitorController() {
        // Simulated controller monitoring logic
        while (true) {
            std::this_thread::sleep_for(100ms);
            // Update joystick and button states here
        }
    }
};

class LocalControllerPublisher : public rclcpp::Node {
public:
    LocalControllerPublisher() : Node("local_pub_node") {
        joy_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        btn_array_pub = this->create_publisher<std_msgs::msg::Int16MultiArray>("/btn_array", 6);
        dpad_array_pub = this->create_publisher<std_msgs::msg::Int16MultiArray>("/dpad_array", 4);

        robot_mode = 0;
        robot_mode_sub = this->create_subscription<std_msgs::msg::Int8>(
            "/robot_mode", 10, [this](const std_msgs::msg::Int8::SharedPtr msg) {
                robot_mode = msg->data;
            });

        timer_ = this->create_wall_timer(
            10ms, std::bind(&LocalControllerPublisher::controllerCallback, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr joy_pub;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr btn_array_pub;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr dpad_array_pub;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr robot_mode_sub;
    rclcpp::TimerBase::SharedPtr timer_;

    XboxController controller;
    int robot_mode;

    void controllerCallback() {
        auto joy_command_msg = geometry_msgs::msg::Twist();
        auto btn_array_msg = std_msgs::msg::Int16MultiArray();
        auto dpad_array_msg = std_msgs::msg::Int16MultiArray();

        auto bumper = controller.readBumper();
        if (bumper == "RB") {
            joy_command_msg.linear.x = controller.readLeftStickY();
            joy_command_msg.angular.z = controller.readRightStickX();
        } else if (bumper == "LB") {
            auto btn_data = controller.readBtn();
            btn_array_msg.data.assign(btn_data.begin(), btn_data.begin() + 6);
            dpad_array_msg.data.assign(btn_data.begin() + 6, btn_data.end());
        }

        if (robot_mode == 0) {
            joy_pub->publish(joy_command_msg);
            dpad_array_pub->publish(dpad_array_msg);
        }

        btn_array_pub->publish(btn_array_msg);
        RCLCPP_INFO(this->get_logger(), "Publishing received joy command: linear.x=%f, angular.z=%f", joy_command_msg.linear.x, joy_command_msg.angular.z);

    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalControllerPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}