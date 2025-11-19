#include <Dynamixel2Arduino.h> // Include Dynamixel2Arduino library
#include <micro_ros_arduino.h> // Include micro-ROS library
#include <rcl/rcl.h> // Include ROS2 core library
#include <rclc/rclc.h> // Include ROS2 client library
#include <rclc/executor.h>  // Include the executor header
#include <std_msgs/msg/int32.h> // Include standard integer message type
#include <geometry_msgs/msg/twist.h>


#define DXL_ID_LEFT 1   // Left wheel motor ID
#define DXL_ID_RIGHT 2  // Right wheel motor ID
#define BAUDRATE 1000000 // Set baud rate for communication
#define DXL_SERIAL Serial2 // Use Serial2 for Dynamixel communication
#define DXL_DIR_PIN 84  // OpenCR direction pin for half-duplex communication

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN); // Create Dynamixel2Arduino object
rclc_support_t support; // Create micro-ROS support object
rcl_allocator_t allocator; // Create allocator for memory management
rcl_node_t node; // Create ROS2 node object
rcl_subscription_t drive_subscription; // Create subscriber for drive messages
geometry_msgs__msg__Twist msgDrive;
rclc_executor_t executor; // Create executor for handling ROS callbacks

void drive_callback(const void *msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    float l = (msg->linear.x - msg->angular.z)/2;
    float r = (msg->linear.x + msg->angular.z)/2;
    dxl.setGoalPWM(DXL_ID_LEFT, l); // Set left motor PWM
    dxl.setGoalPWM(DXL_ID_RIGHT, r); // Set right motor PWM
}

void setup() {
    Serial.begin(115200); // Initialize serial communication
    dxl.begin(BAUDRATE); // Initialize Dynamixel communication
    dxl.setOperatingMode(DXL_ID_LEFT, OP_PWM); // Set left motor to PWM mode
    dxl.setOperatingMode(DXL_ID_RIGHT, OP_PWM); // Set right motor to PWM mode
    dxl.torqueOn(DXL_ID_LEFT); // Enable left motor torque
    dxl.torqueOn(DXL_ID_RIGHT); // Enable right motor torque
    dxl.setGoalPWM(DXL_ID_LEFT, 0); // Stop left motor
    dxl.setGoalPWM(DXL_ID_RIGHT, 0); // Stop right motor

    set_microros_transports(); // Set up micro-ROS communication transport
    allocator = rcl_get_default_allocator(); // Set default allocator
    rclc_support_init(&support, 0, NULL, &allocator); // Initialize ROS2 support
    rclc_node_init_default(&node, "turtlebot_node", "", &support); // Initialize ROS2 node
    rclc_subscription_init_default(&drive_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"); // Create drive subscriber

    // Initialize executor with 1 handle (for the subscriber)
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &drive_subscription, &msgDrive, &drive_callback, ON_NEW_DATA);
}

void loop() {
      // Simple hardcoded PWM test
    dxl.setGoalPWM(DXL_ID_LEFT, 200);   // spin forward
    dxl.setGoalPWM(DXL_ID_RIGHT, 200);
    delay(2000);

    dxl.setGoalPWM(DXL_ID_LEFT, -200);  // spin backward
    dxl.setGoalPWM(DXL_ID_RIGHT, -200);
    delay(2000);

    dxl.setGoalPWM(DXL_ID_LEFT, 0);     // stop
    dxl.setGoalPWM(DXL_ID_RIGHT, 0);
    delay(2000);
    
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); // Process ROS messages
    delay(10); // Small delay to prevent overload
}
