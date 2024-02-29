// -------------------------------------------------------
// ---------- ROS Libraries ------------------------------
// -------------------------------------------------------

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Specific message types
#include <std_msgs/msg/int64.h>

// -------------------------------------------------------
// --------- /ROS Libraries ------------------------------
// -------------------------------------------------------


// ----- Pins -----
// -- Input --
const int rc_input_pin = 5;
const int use_pu_control_pin = 7;

// -- Output --
// Festo
const int control_enable_pin = 0;
const int output_stage_enable_pin = 1;
const int motor_controller_stop_pin = 2;


// ----- Input variables -----
bool use_pu_control = false;
int rc_pwm_input = 1500;
int pu_steering_input = 45;


// ----- Output variables -----
int steering_setpoint = 45;



// ----- Constants -----
// -- RC --
const int min_pwm_input = 1100; // Max left turn
const int max_pwm_input = 1900; // Max right turn
const int middle_pwm_input = 1500; // Straight

// -- PU --
const int pu_min_steering_input = 0; // Max left turn
const int pu_max_steering_input = 90; // Max right turn

// -- Festo --
const int festo_angle_scale_factor = 266; // Scaling factor relating Festo values to degree values from 0-90
const int festo_min_angle = 0 * festo_angle_scale_factor; // Max left turn
const int festo_max_angle = 90 * festo_angle_scale_factor; // Max right turn


// ----- Function variables -----
String setpoint_string;
String angle_string;
int last_angle_setpoint;
const int angle_tolerance = 4 * festo_angle_scale_factor;

// -------------------------------------------------------
// ---------- ROS Necessities ----------------------------
// -------------------------------------------------------

// ------- Subscriber Necessities --------------
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;


// --------- Publisher Necessities -------------
rclc_executor_t executor_pub;
rcl_timer_t timer;

// ---------- Node ----------
rcl_node_t node;
const char * node_name = "lw_steering_portenta_node";
const int number_of_handles = 1; // Sum of subscribers

// ----- Subscribers -----
// -- Steering --
rcl_subscription_t steering_subscriber;
const char * steering_topic_name = "lw_portenta_steering";
std_msgs__msg__Int64 steering_msg;

void steering_subscriber_callback(const void* steering_msg) {
  const std_msgs__msg__Int64* msg = (const std_msgs__msg__Int64*)steering_msg;
  pu_steering_input = msg->data;
}

// ----- Publisher ------

rcl_publisher_t u3_publisher;
const char* u3_topic = "lw_portenta_steering_input";
int64_t u3_msg;

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    std_msgs__msg__Int64 u3_msg;
    u3_msg.data = steering_setpoint; 
    rcl_publish(&u3_publisher, &u3_msg, NULL);
  }
}


// ---------- ROS connection check function ----------
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop() {
  while (1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}




// -------------------------------------------------------
// --------- /ROS Necessities ----------------------------
// -------------------------------------------------------


void setup(){
  
  setupFesto();
  delay(15000);
  setupROS2();

  // -- Input --
  pinMode(use_pu_control_pin, INPUT);
  pinMode(rc_input_pin, INPUT);

  use_pu_control = digitalRead(use_pu_control_pin);
}


void loop(){

  // Spin ROS node
  RCCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(10)));
  
  // Checks if ATV should be controlled using RC or PU
  use_pu_control = digitalRead(use_pu_control_pin);

  // ATV controlled using PU
  if (use_pu_control) {
      // Spin ROS node
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));                                          // Get input
    pu_steering_input = constrain(pu_steering_input, pu_min_steering_input, pu_max_steering_input);         // Constrain input
    steering_setpoint = inputToFestoAngle(pu_steering_input, pu_min_steering_input, pu_max_steering_input); // Map input
  }

  // ATV controlled using RC
  else {
    rc_pwm_input = pulseIn(rc_input_pin, HIGH);                                         // Get input
    rc_pwm_input = constrain(rc_pwm_input, min_pwm_input, max_pwm_input);               // Constrain input
    steering_setpoint = inputToFestoAngle(rc_pwm_input, min_pwm_input, max_pwm_input);  // Map input 
  }

  setSteeringAngle(steering_setpoint);
}


// ----- Functions -----

/*
 * Sets up everything related to ROS2
 */
void setupROS2(){
  // Setup communication via micro-ROS
  set_microros_transports();
  delay(2000);

  // Handles memory allocation for the microcontroller
  allocator = rcl_get_default_allocator();

  // Handles micro-ROS configuration
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Initialise node
  RCCHECK(rclc_node_init_default(&node, node_name, "", &support));

  // ----- Initialise subscriber -----
  RCCHECK(rclc_subscription_init_default(
            &steering_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
            steering_topic_name));

  //------------ Initialise Publisher --------
  rcl_ret_t rc3 = rclc_publisher_init_default(
      &u3_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64), u3_topic);
      const unsigned int timer_timeout = 10;
      RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));



  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, number_of_handles, &allocator));

  // Add handles to executor
  RCCHECK(rclc_executor_add_subscription(&executor, &steering_subscriber, &steering_msg, &steering_subscriber_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));
}

/*
 * Sends formatted order to Festo
 */
void transmitFestoOrder(String order){
  Serial1.println(order);
}

/*
 * Sets up the Festo for communication with the microcontroller
 */
void setupFesto(){
  // Starts serial communication with Festo via TX and RX pins on microcontroller.
  Serial1.begin(9600);

  // Code and comments copied directly from Barholt & Bøe, 2020.
  pinMode(control_enable_pin, OUTPUT);            //Setter pin for "controlEnable" som en output
  pinMode(output_stage_enable_pin, OUTPUT);       //Setter pin for "outputStageEnable" som en output
  pinMode(motor_controller_stop_pin, OUTPUT);     //Setter pin for "mkStop" som en output
  delay(100);                                     //delay for relèene blir aktiverte
  digitalWrite(control_enable_pin, HIGH);         //Aktiviserer "controlEnable" relè
  digitalWrite(output_stage_enable_pin, HIGH);    //Aktiviserer "outputStageEnable" relè
  digitalWrite(motor_controller_stop_pin, HIGH);  //Aktiviserer "mkStop" relè
  // /Code and comments copied directly from Barholt & Bøe, 2020.

  // Enables the Festo to receive steering inputs
  transmitFestoOrder("=651010:0002"); // Festo command: Enable logic
  transmitFestoOrder("=604000:0006"); // Festo command: Shutdown
  transmitFestoOrder("=604000:0007"); // Festo command: Switch on / Disable operation
  transmitFestoOrder("=604000:000F"); // Festo command: Enable operation

  // Homing procedure necessary to find steering angles. Required by Festo
  transmitFestoOrder("=606000:06");   // Festo command: Switch to Homing Mode
  transmitFestoOrder("=604000:001F"); // Festo command: Start Homing run
  delay(20000);
  transmitFestoOrder("=604000:000F"); // Festo command: Enable operation

  // Setup Festo for steering input
  transmitFestoOrder("=606000:01");   // Festo command: Switch to Profile Position Mode
}

/*
 * Modified from Old Code
 */
void setSteeringAngle(int setpoint){

  // Updates steering angle only if the new setpoint deviates from last setpoint more than the angle tolerance.
  if(abs(last_angle_setpoint-setpoint)>angle_tolerance){

    // Format steering angle for Festo
    setpoint_string = formatFestoAngle(setpoint);

    transmitFestoOrder("=604000:000F");             // Festo command: Enable operation
    transmitFestoOrder("=607A00:"+setpoint_string); // Festo command: Sets target position
    transmitFestoOrder("=604000:009F");             // Festo command: Go to target position

    last_angle_setpoint = setpoint; // Save setpoint for filtering

  }
}

int inputToFestoAngle(int input_signal, int min_input, int max_input) {
  return map(input_signal, min_input, max_input, festo_min_angle, festo_max_angle);
}

String formatFestoAngle(int angle){
  angle_string = String(angle);
    while(angle_string.length() < 8){
      angle_string = '0'+angle_string;
    }
  return angle_string;
}
