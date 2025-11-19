#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <Servo.h>

rcl_subscription_t controllerSubDpadArray;
rcl_subscription_t controllerSubBtnArray;
rcl_subscription_t controllerSubDrive;

rcl_timer_t timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
bool micro_ros_init_successful;

std_msgs__msg__Int16MultiArray msgBtnArray;
std_msgs__msg__Int16MultiArray msgDpadArray;
geometry_msgs__msg__Twist msgDrive;


//Servo Setup
Servo servo_drive_right;
Servo servo_drive_left;
Servo servo_conveyor_belt;
Servo servo_digger_pivot;
Servo servo_digger_slide;
Servo servo_digger_dig;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
}

#define servo_drive_left_pin 5
#define servo_drive_right_pin 6
#define servo_conveyor_belt_pin 10
#define servo_digger_pivot_pin 3
#define servo_digger_slide_pin 11
#define servo_digger_dig_pin 9

#define BDPIN_LED_USER_1        22
#define BDPIN_LED_USER_2        23
#define BDPIN_LED_USER_3        24
#define BDPIN_LED_USER_4        25

int led_pin_user[4] = { BDPIN_LED_USER_1, BDPIN_LED_USER_2, BDPIN_LED_USER_3, BDPIN_LED_USER_4 };

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
const int DEBOUNCE_DELAY_MS = 200;
unsigned long lastDebounceTime = 0;

int PWM_MIN = 800;
int PWMRANGE = 2200;
float IN_MIN = -1.0;
float IN_MAX = 1.0;
float OUT_MIN_DRIVE = 1300;
float OUT_MAX_DRIVE = 1700;

int NEO_MIN = 1400;
int NEO_MAX = 1600;
int PWM_OFF = 1500;

void error_loop(){
  while(1){
    int i;
    for( i=0; i<4; i++ )
    {
    digitalWrite(led_pin_user[i], HIGH);
    delay(100);
    }
    for( i=0; i<4; i++ )
    {
    digitalWrite(led_pin_user[i], LOW);
    delay(100);
    }

    delay(100);
  }
}


float mapFloat(float x, float in_min, float in_max, float OUT_MIN_DRIVE, float OUT_MAX_DRIVE)
{
return (x - in_min) * (OUT_MAX_DRIVE - OUT_MIN_DRIVE) / (in_max - in_min) + OUT_MIN_DRIVE;
}


static bool digger_state = false;
static bool conveyor_state = false;
static bool auto_dig = false;
static bool auto_dump = false;
static bool auto_dig_dump = false;

void BtnArray_callback(const void * msgin)
 {  
   const std_msgs__msg__Int16MultiArray * msg = (const std_msgs__msg__Int16MultiArray *)msgin;
   unsigned long currentMillis = millis();

   //Button X Callback Keep On
  if (msg->data.data[0]== 1 && !digger_state && (currentMillis - lastDebounceTime) > DEBOUNCE_DELAY_MS){
    servo_digger_dig.writeMicroseconds(NEO_MIN);
    digger_state = true;
    lastDebounceTime = currentMillis;
  }
  else if(msg->data.data[0] == 1 && digger_state && (currentMillis - lastDebounceTime) > DEBOUNCE_DELAY_MS){
    servo_digger_dig.writeMicroseconds(PWM_OFF);
    digger_state = false;
    lastDebounceTime = currentMillis;
  }
  
   //Button Y Callback 
  if (msg->data.data[1]== 1 && !conveyor_state && (currentMillis - lastDebounceTime) > DEBOUNCE_DELAY_MS){
    servo_conveyor_belt.writeMicroseconds(1300);
    conveyor_state = true;
    lastDebounceTime = currentMillis;
  }
  else if(msg->data.data[1] == 1 && conveyor_state && (currentMillis - lastDebounceTime) > DEBOUNCE_DELAY_MS){
    servo_conveyor_belt.writeMicroseconds(PWM_OFF);
    conveyor_state = false;
    lastDebounceTime = currentMillis;
  }
   
  //Button A Callback (autonomous digging)
  if (msg->data.data[2]== 1 && !auto_dig && (currentMillis - lastDebounceTime) > DEBOUNCE_DELAY_MS){
    auto_dig = true;
    unsigned long startTime;
    startTime = millis(); // Record the start time
    // dig for 4 seconds
    while (millis() - startTime < 4000){
      servo_digger_dig.writeMicroseconds(NEO_MIN);
    }
    servo_digger_dig.writeMicroseconds(PWM_OFF);
    startTime = millis();

    // slide digger down for 1 second
    while (millis() - startTime < 100 && auto_dig){
      servo_digger_slide.writeMicroseconds(NEO_MIN);
    }
    servo_digger_slide.writeMicroseconds(PWM_OFF);

    //Dig for 4 seconds
    startTime = millis();
    while (millis() - startTime < 4000 && auto_dig){
      servo_digger_dig.writeMicroseconds(NEO_MIN);
    }

    servo_digger_dig.writeMicroseconds(PWM_OFF);
    servo_digger_slide.writeMicroseconds(PWM_OFF);
    auto_dig = false;
  }

  else if (msg->data.data[2]== 1 && auto_dig && (currentMillis - lastDebounceTime) > DEBOUNCE_DELAY_MS){
    auto_dig = false;
    servo_digger_dig.writeMicroseconds(PWM_OFF);
    servo_digger_slide.writeMicroseconds(NEO_MIN);
  }
  
  //Button B Callback 
  if (msg->data.data[3]== 1 && !auto_dump && (currentMillis - lastDebounceTime) > DEBOUNCE_DELAY_MS){
    auto_dump = true;
    unsigned long startTime;
    startTime = millis(); // Record the start time

    // conveyor belt for .5 seconds
    while (millis() - startTime < 500){
      servo_conveyor_belt.writeMicroseconds(1300);
    }
    servo_conveyor_belt.writeMicroseconds(PWM_OFF);
    delay(1000);
    startTime = millis();

    // conveyor belt for .5 seconds
    while (millis() - startTime < 500 && auto_dig){
      servo_conveyor_belt.writeMicroseconds(1300);
    }
    servo_conveyor_belt.writeMicroseconds(PWM_OFF);
    delay(1000);
    startTime = millis();

    // conveyor belt for .5 seconds
    while (millis() - startTime < 500 && auto_dig){
      servo_conveyor_belt.writeMicroseconds(1300);
    }
    servo_conveyor_belt.writeMicroseconds(PWM_OFF);

    auto_dump = false;
  }

  else if (msg->data.data[3]== 0 && auto_dump && (currentMillis - lastDebounceTime) > DEBOUNCE_DELAY_MS){
    servo_conveyor_belt.writeMicroseconds(PWM_OFF);
  }
   
   //Button Select Callback (begin auto dig and dump sequence)
  if (msg->data.data[4]== 1 && !auto_dig_dump && (currentMillis - lastDebounceTime) > DEBOUNCE_DELAY_MS){
    auto_dig_dump = true;
    unsigned long startTime;
    startTime = millis(); // Record the start time

    //DIGGING PORTION
    // ==================================================  
    // dig for 4 seconds
    while (millis() - startTime < 4000){
      servo_digger_dig.writeMicroseconds(NEO_MIN);
    }
    servo_digger_dig.writeMicroseconds(PWM_OFF);
    startTime = millis();

    // slide digger down for 1 second
    while (millis() - startTime < 100){
      servo_digger_slide.writeMicroseconds(NEO_MIN);
    }
    servo_digger_slide.writeMicroseconds(PWM_OFF);

    //Dig for 4 seconds
    startTime = millis();
    while (millis() - startTime < 4000){
      servo_digger_dig.writeMicroseconds(NEO_MIN);
    }
    delay(500);
    startTime = millis();

    // slide digger back up
    while (millis() - startTime < 800){
      servo_digger_slide.writeMicroseconds(NEO_MAX);
    }
    servo_digger_slide.writeMicroseconds(PWM_OFF);

    servo_digger_dig.writeMicroseconds(PWM_OFF);
    servo_digger_slide.writeMicroseconds(PWM_OFF);

    //DRIVING PORTION
    //=============================================
    delay(500);
    startTime = millis(); // Record the start time

    // drive backward for 1 second.
    while (millis() - startTime < 1000){
      servo_drive_left.writeMicroseconds(1700);
      servo_drive_right.writeMicroseconds(1300);
    }
    servo_drive_left.writeMicroseconds(PWM_OFF);
    servo_drive_right.writeMicroseconds(PWM_OFF);
    delay(500);

    //DUMPING PORTION
    //=============================================
    startTime = millis(); // Record the start time

    // conveyor belt for .5 seconds
    while (millis() - startTime < 1000){
      servo_conveyor_belt.writeMicroseconds(1300);
    }
    servo_conveyor_belt.writeMicroseconds(PWM_OFF);
    delay(1000);
    startTime = millis();

    // conveyor belt for .5 seconds
    while (millis() - startTime < 1000){
      servo_conveyor_belt.writeMicroseconds(1300);
    }
    servo_conveyor_belt.writeMicroseconds(PWM_OFF);
    delay(1000);
    startTime = millis();

    // conveyor belt for .5 seconds
    while (millis() - startTime < 1000){
      servo_conveyor_belt.writeMicroseconds(1300);
    }
    servo_conveyor_belt.writeMicroseconds(PWM_OFF);

  }
  else if (msg->data.data[4]== 1 && auto_dig_dump && (currentMillis - lastDebounceTime) > DEBOUNCE_DELAY_MS){
    auto_dig_dump = false;

  }
   
   //Button Start Callback 
   if (msg->data.data[5]== 1){
    digitalWrite(led_pin_user[1], LOW);

   }
   else if (msg->data.data[5]== 0&&msg->data.data[4]== 0){
    digitalWrite(led_pin_user[1], HIGH);
   }
 }

void DpadArray_callback(const void * msgin)
  {  
   const std_msgs__msg__Int16MultiArray * msg = (const std_msgs__msg__Int16MultiArray *)msgin;
   
  //  //Dpad Up Callback
  //  if (msg->data.data[0]== 1){
  //   servo_digger_pivot.writeMicroseconds(NEO_MAX);
  //  }

  //  //Dpad Down Callback
  //  if (msg->data.data[1]== 1){
  //   servo_digger_pivot.writeMicroseconds(NEO_MIN);
  //  }
   
  //  if (msg->data.data[1]== 0&&msg->data.data[0]== 0){
  //   servo_digger_pivot.writeMicroseconds(NEO_MAX);
  //  }

   //Dpad Left Callback
   if (msg->data.data[2]== 1){
    servo_digger_slide.writeMicroseconds(NEO_MIN);
   }
   
   //Dpad Right Callback
   if (msg->data.data[3]== 1){
    servo_digger_slide.writeMicroseconds(NEO_MAX);
   }
   
   if (msg->data.data[3]== 0 && msg->data.data[2]== 0){
    servo_digger_slide.writeMicroseconds(PWM_OFF);
   }
 }

// Define variables for gradually changing the PWM signal
uint16_t currentLeftPWM = 1500;
uint16_t currentRightPWM = 1500;
const int accelerationStep = 10;  // Adjust this value to control the acceleration rate

void Drive_callback(const void *msgin)
{  

  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  float l = (msg->linear.x - msg->angular.z)/2;
  float r = (msg->linear.x + msg->angular.z)/2;

  r = -r;
  
  uint16_t targetLeftPWM = mapFloat(l, IN_MIN, IN_MAX, OUT_MIN_DRIVE, OUT_MAX_DRIVE);
  uint16_t targetRightPWM  = mapFloat(r, IN_MIN, IN_MAX, OUT_MIN_DRIVE, OUT_MAX_DRIVE);

  // Gradually change the PWM values towards the target values
  if (targetLeftPWM > currentLeftPWM) {
    currentLeftPWM = min(currentLeftPWM + accelerationStep, targetLeftPWM);
  } else if (targetLeftPWM < currentLeftPWM) {
    currentLeftPWM = max(currentLeftPWM - accelerationStep, targetLeftPWM);
  }

  if (targetRightPWM > currentRightPWM) {
    currentRightPWM = min(currentRightPWM + accelerationStep, targetRightPWM);
  } else if (targetRightPWM < currentRightPWM) {
    currentRightPWM = max(currentRightPWM - accelerationStep, targetRightPWM);
  }

  // Set the PWM signal for the motors
  servo_drive_left.writeMicroseconds(currentLeftPWM);
  servo_drive_right.writeMicroseconds(currentRightPWM);
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();


  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber btn_array
  RCCHECK(rclc_subscription_init_default(
    &controllerSubBtnArray,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "/btn_array"));
  // create subscriber dpad_array
  RCCHECK(rclc_subscription_init_default(
    &controllerSubDpadArray,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "/dpad_array"));

  // create subscriber /cmd_vel
  RCCHECK(rclc_subscription_init_default(
    &controllerSubDrive,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));

  //Not sure if we need this timer, but keeping it in just in case
  //create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 8, &allocator));

  // add subscriptions to executor
  RCCHECK(rclc_executor_add_subscription(&executor, &controllerSubDrive, &msgDrive, &Drive_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &controllerSubBtnArray, &msgBtnArray, &BtnArray_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &controllerSubDpadArray, &msgDpadArray, &DpadArray_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  return true;  
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  set_microros_transports();
  state = WAITING_AGENT;
  pinMode(led_pin_user[0], OUTPUT);
  pinMode(led_pin_user[1], OUTPUT);
  pinMode(led_pin_user[2], OUTPUT);
  pinMode(led_pin_user[3], OUTPUT);
  delay(2000);

  // rageboard code for motor drivers
  servo_drive_left.attach(servo_drive_left_pin);
  servo_drive_right.attach(servo_drive_right_pin);
  servo_conveyor_belt.attach(servo_conveyor_belt_pin);
  servo_digger_pivot.attach(servo_digger_pivot_pin);
  servo_digger_slide.attach(servo_digger_slide_pin);
  servo_digger_dig.attach(servo_digger_dig_pin);
  

  
  servo_drive_left.writeMicroseconds(1500);
  servo_drive_right.writeMicroseconds(1500);
  servo_conveyor_belt.writeMicroseconds(1500);
  servo_digger_pivot.writeMicroseconds(1500);
  servo_digger_slide.writeMicroseconds(1500);
  servo_digger_dig.writeMicroseconds(1500);
  
  delay(1000);
  micro_ros_init_successful = true;

  // Need to have this wacky chat gpt code for subscribing to int16multiarray type messages
  msgBtnArray.data.capacity = 100;
  msgBtnArray.data.data = (int16_t*) malloc(msgBtnArray.data.capacity * sizeof(int16_t));
  msgBtnArray.data.size = 0;
  
  // Assigning static memory to the sequence
  static int16_t memory[100];
  msgBtnArray.data.capacity = 100;
  msgBtnArray.data.data = memory;
  msgBtnArray.data.size = 0;
  
  // Filling some data
  for(int16_t i = 0; i < 3; i++){
    msgBtnArray.data.data[i] = i;
    msgBtnArray.data.size++;
  }
  
  // Need to have this wacky chat gpt code for subscribing to int16multiarray type messages
  msgDpadArray.data.capacity = 100;
  msgDpadArray.data.data = (int16_t*) malloc(msgDpadArray.data.capacity * sizeof(int16_t));
  msgDpadArray.data.size = 0;
  
  // Assigning static memory to the sequence
  //static int16_t memory[100];
  msgDpadArray.data.capacity = 100;
  msgDpadArray.data.data = memory;
  msgDpadArray.data.size = 0;
  
  // Filling some data
  for(int16_t i = 0; i < 5; i++){
    msgDpadArray.data.data[i] = i;
    msgDpadArray.data.size++;
  }
}

void loop() {
  switch(state){
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

}