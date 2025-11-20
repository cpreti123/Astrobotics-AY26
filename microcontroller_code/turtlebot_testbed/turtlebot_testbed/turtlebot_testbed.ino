#include <Dynamixel2Arduino.h> // Include Dynamixel2Arduino library
#include <micro_ros_arduino.h> // Include micro-ROS library
#include <rcl/rcl.h> // Include ROS2 core library
#include <rclc/rclc.h> // Include ROS2 client library
#include <rclc/executor.h>  // Include the executor header
#include <std_msgs/msg/int32.h> // Include standard integer message type
#include <geometry_msgs/msg/twist.h>


#include "C:/Users/rylan.pettus/OneDrive - West Point/AY 2025-2/Astrobotics/Astro-AY25/microcontroller_code/turtlebot_testbed/turtlebot3_motor_driver.h"

// Limit values (XM430-W210-T and XM430-W350-T)
// MAX RPM is 77 when DXL is powered 12.0V
// 77 / 0.229 (RPM) = 336.24454...
const uint16_t LIMIT_X_MAX_VELOCITY = 337; 
// V = r * w = r     *        (RPM             * 0.10472)
//           = 0.033 * (0.229 * Goal_Velocity) * 0.10472
// Goal_Velocity = V * 1263.632956882
const float VELOCITY_CONSTANT_VALUE = 1263.632956882; 

/* DYNAMIXEL Information for controlling motors and  */
const uint8_t DXL_MOTOR_ID_LEFT = 1; // ID of left motor
const uint8_t DXL_MOTOR_ID_RIGHT = 2; // ID of right motor
const float DXL_PORT_PROTOCOL_VERSION = 2.0; // Dynamixel protocol version 2.0
const uint32_t DXL_PORT_BAUDRATE = 1000000; // baurd rate of Dynamixel
const int OPENCR_DXL_DIR_PIN = 84; // Arduino pin number of DYNAMIXEL direction pin on OpenCR.

ParamForSyncReadInst_t sync_read_param;
ParamForSyncWriteInst_t sync_write_param;
RecvInfoFromStatusInst_t read_result;
Dynamixel2Arduino dxl(Serial3, OPENCR_DXL_DIR_PIN);

// constructor
Turtlebot3MotorDriver::Turtlebot3MotorDriver()
: left_wheel_id_(DXL_MOTOR_ID_LEFT),
  right_wheel_id_(DXL_MOTOR_ID_RIGHT),
  torque_(false)
{
}

//destructor
Turtlebot3MotorDriver::~Turtlebot3MotorDriver()
{
  close();
  digitalWrite(BDPIN_DXL_PWR_EN, LOW);
}
// rclc_support_t support; // Create micro-ROS support object
// rcl_allocator_t allocator; // Create allocator for memory management
// rcl_node_t node; // Create ROS2 node object
// rcl_subscription_t drive_subscription; // Create subscriber for drive messages
// geometry_msgs__msg__Twist msgDrive;
// rclc_executor_t executor; // Create executor for handling ROS callbacks

// void drive_callback(const void *msgin) {
//     const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
//     float l = (msg->linear.x - msg->angular.z)/2;
//     float r = (msg->linear.x + msg->angular.z)/2;
//     dxl.setGoalPWM(DXL_ID_LEFT, l); // Set left motor PWM
//     dxl.setGoalPWM(DXL_ID_RIGHT, r); // Set right motor PWM
// }

bool Turtlebot3MotorDriver::set_torque(bool onoff)
{
  bool ret = false;

  sync_write_param.addr = 64;
  sync_write_param.length = 1;
  sync_write_param.xel[LEFT].data[0] = onoff;
  sync_write_param.xel[RIGHT].data[0] = onoff;

  if(dxl.syncWrite(sync_write_param) == true){
    ret = true;
    torque_ = onoff;
  }

  return ret;
}

void Turtlebot3MotorDriver::close(void)
{
  // Disable Dynamixel Torque
  set_torque(false);
}

bool write_velocity(int32_t left_value, int32_t right_value)
{
  bool ret = false;

  sync_write_param.addr = 104;
  sync_write_param.length = 4;
  memcpy(sync_write_param.xel[LEFT].data, &left_value, sync_write_param.length);
  memcpy(sync_write_param.xel[RIGHT].data, &right_value, sync_write_param.length);

  if(dxl.syncWrite(sync_write_param)){
    ret = true;
  }

  return ret;
}

bool write_profile_acceleration(uint32_t left_value, uint32_t right_value)
{
  bool ret = false;

  sync_write_param.addr = 108;
  sync_write_param.length = 4;
  memcpy(sync_write_param.xel[LEFT].data, &left_value, sync_write_param.length);
  memcpy(sync_write_param.xel[RIGHT].data, &right_value, sync_write_param.length);

  if(dxl.syncWrite(sync_write_param)){
    ret = true;
  }

  return ret;
}

bool control_motors(const float wheel_separation, float linear_value, float angular_value)
{
  bool dxl_comm_result = false;
  
  float wheel_velocity[MortorLocation::MOTOR_NUM_MAX];
  float lin_vel = linear_value;
  float ang_vel = angular_value;

  wheel_velocity[LEFT]   = lin_vel - (ang_vel * wheel_separation / 2);
  wheel_velocity[RIGHT]  = lin_vel + (ang_vel * wheel_separation / 2);

  wheel_velocity[LEFT]  = constrain(wheel_velocity[LEFT]  * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
  wheel_velocity[RIGHT] = constrain(wheel_velocity[RIGHT] * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);

  dxl_comm_result = write_velocity((int32_t)wheel_velocity[LEFT], (int32_t)wheel_velocity[RIGHT]);
  if (dxl_comm_result == false)
    return false;

  return true;
}

bool Turtlebot3MotorDriver::init(void)
{
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  drv_dxl_init();

  dxl.begin(DXL_PORT_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PORT_PROTOCOL_VERSION);

  sync_write_param.id_count = 2;
  sync_write_param.xel[LEFT].id = left_wheel_id_;
  sync_write_param.xel[RIGHT].id = right_wheel_id_;

  sync_read_param.addr = 132;
  sync_read_param.length = 4;
  sync_read_param.id_count = 2;
  sync_read_param.xel[LEFT].id = left_wheel_id_;
  sync_read_param.xel[RIGHT].id = right_wheel_id_;

  // Enable Dynamixel Torque
  set_torque(true);

  return true;
}

void setup() {

  Turtlebot3MotorDriver motorDriver;
  motorDriver.init();

  // dxl.setGoalPWM(DXL_ID_LEFT, 1500); // Stop left motor
  // dxl.setGoalPWM(DXL_ID_RIGHT, 1500); // Stop right motor

  // set_microros_transports(); // Set up micro-ROS communication transport
  // allocator = rcl_get_default_allocator(); // Set default allocator
  // rclc_support_init(&support, 0, NULL, &allocator); // Initialize ROS2 support
  // rclc_node_init_default(&node, "turtlebot_node", "", &support); // Initialize ROS2 node
  // rclc_subscription_init_default(&drive_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"); // Create drive subscriber

  // // Initialize executor with 1 handle (for the subscriber)
  // rclc_executor_init(&executor, &support.context, 1, &allocator);
  // rclc_executor_add_subscription(&executor, &drive_subscription, &msgDrive, &drive_callback, ON_NEW_DATA);
  
  // for (int id = 1; id <= 10; id++) {
  //     if (dxl.ping(id)) {
  //         Serial.print("Found motor at ID: ");
  //         Serial.println(id);
  //     }
  // }

}

void loop() {
  write_velocity(LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
  // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); // Process ROS messages
  // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); // Process ROS messages
}
