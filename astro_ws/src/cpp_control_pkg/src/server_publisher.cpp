#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <vector>

using std::placeholders::_1;

class Button
{
public:
    Button(rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher)
        : publisher_(publisher) {}

    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    std_msgs::msg::Int16MultiArray msg_;
};

class ControllerPublisher : public rclcpp::Node
{
public:
    ControllerPublisher()
        : Node("controller_pub_node"), robot_mode_(0)
    {
        joy_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        btn_array_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("/btn_array", 6);
        dpad_array_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("/dpad_array", 4);

        robot_mode_sub_ = this->create_subscription<std_msgs::msg::Int8>("/robot_mode", 10, std::bind(&ControllerPublisher::robot_mode_callback, this, _1));

        btn_array_ = std::make_shared<Button>(btn_array_pub_);
        dpad_array_ = std::make_shared<Button>(dpad_array_pub_);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&ControllerPublisher::controllerpub_callback, this));

        setup_socket();
    }

private:
    void robot_mode_callback(const std_msgs::msg::Int8::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received robot mode: %d", msg->data);
        robot_mode_ = msg->data;
    }

    void controllerpub_callback()
    {
        try
        {
            char buffer[1024];
            int bytes_received = recv(client_socket_, buffer, sizeof(buffer), 0);
            if (bytes_received > 0)
            {
                std::vector<float> joy_list(2);
                std::vector<int16_t> button_list;
                std::vector<int16_t> dpad_list;

                // Deserialize data from buffer (use your serialization/deserialization library here)
                // For example, use the msgpack or protobuf libraries to do so

                geometry_msgs::msg::Twist joy_command;
                joy_command.linear.x = joy_list[0];
                joy_command.angular.z = joy_list[1];

                if (robot_mode_ == 0)
                {
                    joy_pub_->publish(joy_command);
                    RCLCPP_INFO(this->get_logger(), "Publishing received joy command: linear.x=%f, angular.z=%f", joy_command.linear.x, joy_command.angular.z);

                    dpad_array_->msg_.data = dpad_list;
                    dpad_array_->publisher_->publish(dpad_array_->msg_);
                }

                btn_array_->msg_.data = button_list;
                btn_array_->publisher_->publish(btn_array_->msg_);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
            close(client_socket_);
            setup_socket();
        }
    }

    void setup_socket()
    {
        server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_socket_ == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return;
        }

        sockaddr_in server_address;
        server_address.sin_family = AF_INET;
        server_address.sin_port = htons(1234);
        server_address.sin_addr.s_addr = INADDR_ANY;

        if (bind(server_socket_, (struct sockaddr *)&server_address, sizeof(server_address)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Bind failed");
            return;
        }

        if (listen(server_socket_, 1) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Listen failed");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Waiting for client connection...");
        client_socket_ = accept(server_socket_, nullptr, nullptr);
        if (client_socket_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Accept failed");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Connection established");
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr joy_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr btn_array_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr dpad_array_pub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr robot_mode_sub_;

    std::shared_ptr<Button> btn_array_;
    std::shared_ptr<Button> dpad_array_;

    rclcpp::TimerBase::SharedPtr timer_;
    int robot_mode_;
    int server_socket_;
    int client_socket_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto controller_pub = std::make_shared<ControllerPublisher>();
    rclcpp::spin(controller_pub);
    rclcpp::shutdown();
    return 0;
}
