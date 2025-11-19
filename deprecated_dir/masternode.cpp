#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>

class Button {
public:
    Button(int channel, std::string msg_type) : channel(channel), msg_type(msg_type) {}

private:
    int channel;
    std::string msg_type;
};

class MasterNode : public rclcpp::Node {
public:
    MasterNode() : Node("master_node"), robot_mode_(0) {
        // Initialize publisher for robot mode
        robot_mode_pub_ = this->create_publisher<std_msgs::msg::Int8>("robot_mode", 10);

        // Initialize subscription to button array

        subscription_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "btn_array", 10, std::bind(&MasterNode::btn_array_callback, this, std::placeholders::_1)
        );

        // Timer to call the robot mode callback
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&MasterNode::robot_mode_callback, this)
        );
    }

private:
    int robot_mode_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr robot_mode_pub_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Callback function for button array messages
    void btn_array_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        if (msg && !msg->data.empty()) {
            if (msg->data[2] == 1) {  // A button
                RCLCPP_INFO(this->get_logger(), "Turning on teleop mode");
                robot_mode_ = 0;  // teleop mode
            } else if (msg->data[3] == 1) {  // B button
                RCLCPP_INFO(this->get_logger(), "Turning on digging mode");
                robot_mode_ = 1;  // dig mode
            } else if (msg->data[1] == 1) {  // Y button
                RCLCPP_INFO(this->get_logger(), "Turning on dumping mode");
                robot_mode_ = 2;  // dump mode
            }
        }
    }

    // Callback function for robot mode
    void robot_mode_callback() {
        auto msg = std_msgs::msg::Int8();
        msg.data = robot_mode_;
        robot_mode_pub_->publish(msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MasterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}