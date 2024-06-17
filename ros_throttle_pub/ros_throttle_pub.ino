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
const int emergency_stop_pin = 6;
const int use_pu_control_pin = 7;
const int rear_brake_position_pin = A0;
const int front_brake_position_pin = A1;

// -- Output --
const int rear_brake_backward_pin = 0;
const int rear_brake_forward_pin = 1;
const int front_brake_backward_pin = 2;
const int front_brake_forward_pin = 3;
const int throttle_output_pin = A6;  // DAC pin for Portenta H7, only works on A6.


// ----- Output variables -----
int throttle_output = 0;
int brake_output = 0;


// ----- Input variables -----
bool use_pu_control = false;
int rc_pwm_input = 1500;
int pu_throttle_input = 0;
int pu_brake_input = 0;

// ----- Message types -----
std_msgs__msg__Int64 u1_msg;
std_msgs__msg__Int64 u2_msg;
std_msgs__msg__Int64 u3_msg;
/*

*/
// ----- Constants -----
// -- RC --
const int min_pwm_input = 1000;     // Max brake
const int max_pwm_input = 2000;     // Max Throttle
const int middle_pwm_input = 1500;  // No brake and no throttle

// -- PU --
const int pu_min_throttle_input = 0;
const int pu_max_throttle_input = 100;
const int pu_min_brake_input = 0;
const int pu_max_brake_input = 100;

// -- Actuator --
const int min_throttle_output = 60;   // 0.7V on throttle_output_pin ----- 0;   // 0V on throttle_output_pin
const int max_throttle_output = 102;  // 1.2V on throttle_output_pin ----- 255; // 3V on throttle_output_pin
const int min_brake_output = 125;     // 175 test value -> Test 125 first, then increase towards 175 if possible
const int max_brake_output = 300;     // 450 test value -> Test 300 first, then increase towards 450 if possible. BE CAREFUL AS THIS MIGHT BREAK THE HANDLES
const int brake_tolerance = 10;       // Brake tolerance for brake setpoint position vs actual position of brake


// ----- Function variables -----
int rear_brake_position;
int front_brake_position;
int current_rear_error;
int current_front_error;

// ----- Enums -----
enum Brake {
  rear,
  front,
  both
};

enum BrakeState {
  push,
  pull,
  stay
};


void setBrakeState(Brake brake, BrakeState brake_state);


// -------------------------------------------------------
// ---------- ROS Necessities ----------------------------
// -------------------------------------------------------
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;


// publisher

rclc_executor_t executor_pub;
rcl_timer_t timer;




// ---------- Node ----------
rcl_node_t node;
const int number_of_handles = 2;  // Sum of subscribers

// ----- Subscribers -----
// -- Throttle --
rcl_subscription_t throttle_subscriber;
std_msgs__msg__Int64 throttle_msg;

void throttle_subscriber_callback(const void* throttle_msg) {
  const std_msgs__msg__Int64* msg = (const std_msgs__msg__Int64*)throttle_msg;
  pu_throttle_input = msg->data;
}

// -- Brake --
rcl_subscription_t brake_subscriber;
std_msgs__msg__Int64 brake_msg;

//publishers

rcl_publisher_t u1_publisher;
const char* u1_topic = "lw_portenta_throttle_input";
// int64_t u1_msg;

rcl_publisher_t u2_publisher;
const char* u2_topic = "lw_portenta_brake_input";
// int64_t u2_msg;

// rcl_publisher_t u3_publisher;
// const char* u3_topic = "lw_portenta_steering_input";
// int64_t u3_msg;

void brake_subscriber_callback(const void* brake_msg) {
  const std_msgs__msg__Int64* msg = (const std_msgs__msg__Int64*)brake_msg;
  pu_brake_input = msg->data;
}








void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // std_msgs__msg__Int64 u1_msg;  //Trengte ikke disse
    // std_msgs__msg__Int64 u2_msg;
    // std_msgs__msg__Int64 u3_msg;

    u1_msg.data = throttle_output;
    u2_msg.data = brake_output;
    // Anta at du har en variabel for styringsinput som du ønsker å publisere
    //u3_msg.data = steering_input;

    rcl_publish(&u1_publisher, &u1_msg, NULL);
    rcl_publish(&u2_publisher, &u2_msg, NULL);
    // rcl_publish(&u3_publisher, &u3_msg, NULL);
  }
}



// ---------- ROS connection check function ----------
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }

void error_loop() {
  while (1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); //comment this
    delay(100);
  }
}

// -------------------------------------------------------
// --------- /ROS Necessities ----------------------------
// -------------------------------------------------------


void setup() {


  setupROS2();

  // -- Input --
  pinMode(emergency_stop_pin, INPUT);
  pinMode(rc_input_pin, INPUT);
  pinMode(rear_brake_position_pin, INPUT);
  pinMode(front_brake_position_pin, INPUT);
  pinMode(use_pu_control_pin, INPUT);

  // -- Output --
  pinMode(throttle_output_pin, OUTPUT);
  pinMode(rear_brake_backward_pin, OUTPUT);
  pinMode(rear_brake_forward_pin, OUTPUT);
  pinMode(front_brake_backward_pin, OUTPUT);
  pinMode(front_brake_forward_pin, OUTPUT);

  use_pu_control = digitalRead(use_pu_control_pin);
}

void loop() {

  // Spin ROS node

  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  RCCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(10)));


  u1_msg.data = throttle_output;  //This had to be u1_msg.data!
  rcl_publish(&u1_publisher, &u1_msg, NULL);



  // ATV Emergency Stop is pressed
  if (digitalRead(emergency_stop_pin)) { //interrupt

    do {
      updateThrottle(min_throttle_output);
      updateBrakes(max_brake_output);
      u1_msg.data = 777;  //This had to be u1_msg.data!
      rcl_publish(&u1_publisher, &u1_msg, NULL);
    } while (digitalRead(emergency_stop_pin));  //Waits for brakes to reach max brake state

    setBrakeState(Brake::both, BrakeState::stay);  //Avoid brakes destroying ATV

    while (true) {
      u1_msg.data = 666;  //This had to be u1_msg.data!
      rcl_publish(&u1_publisher, &u1_msg, NULL);
    }
  }



  // Checks if ATV should be controlled using RC or PU
  use_pu_control = digitalRead(use_pu_control_pin);

  // ATV controlled using RC
  if (!use_pu_control) {

    rc_pwm_input = pulseIn(rc_input_pin, HIGH);
    rc_pwm_input = constrain(rc_pwm_input, min_pwm_input, max_pwm_input);
    //   std_msgs__msg__Int64 u1_msg;
    //   std_msgs__msg__Int64 u2_msg;
    //   std_msgs__msg__Int64 u3_msg;
    if (rc_pwm_input > middle_pwm_input) {
      throttle_output = map(rc_pwm_input, middle_pwm_input, max_pwm_input, min_throttle_output, max_throttle_output);
      updateBrakes(min_brake_output);
      updateThrottle(throttle_output);
      u1_msg.data = throttle_output;  //This had to be u1_msg.data!
      rcl_publish(&u1_publisher, &u1_msg, NULL);
    } else if (rc_pwm_input <= middle_pwm_input) {
      brake_output = map(rc_pwm_input, middle_pwm_input, min_pwm_input, min_brake_output, max_brake_output);
      updateThrottle(min_throttle_output);
      updateBrakes(brake_output);
      u2_msg.data = brake_output;
      rcl_publish(&u2_publisher, &u2_msg, NULL);
    } else {
      updateThrottle(min_throttle_output);
      updateBrakes(max_brake_output);
    }
  }

  // ATV controlled using PU
  else {
    pu_throttle_input = constrain(pu_throttle_input, pu_min_throttle_input, pu_max_throttle_input);
    pu_brake_input = constrain(pu_brake_input, pu_min_brake_input, pu_max_brake_input);

    throttle_output = map(pu_throttle_input, pu_min_throttle_input, pu_max_throttle_input, min_throttle_output, max_throttle_output);
    brake_output = map(pu_brake_input, pu_min_brake_input, pu_max_brake_input, min_brake_output, max_brake_output);

    updateBrakes(brake_output);
    updateThrottle(throttle_output);
  }
}


// ----- Functions -----

/*
   Sets up everything related to ROS2
*/
void setupROS2() {
  // Setup communication via micro-ROS
  set_microros_transports();
  delay(2000);

  // Handles memory allocation for the microcontroller
  allocator = rcl_get_default_allocator();

  // Handles micro-ROS configuration
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Initialise node
  RCCHECK(rclc_node_init_default(&node, "lw_throttle_brake_portenta_node", "", &support));

  // ----- Initialise subscribers -----
  // -- Throttle --
  RCCHECK(rclc_subscription_init_default(
            &throttle_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
            "lw_portenta_throttle_subscriber"));

  // -- Brake --
  RCCHECK(rclc_subscription_init_default(
            &brake_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
            "lw_portenta_brake_subscriber"));


  // creating pubs
  // Initialisering av publishere
  rcl_ret_t rc1 = rclc_publisher_init_default(
                    &u1_publisher, &node,
                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64), u1_topic);


  rcl_ret_t rc2 = rclc_publisher_init_default(
                    &u2_publisher, &node,
                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64), u2_topic);

  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(timer_timeout),
            timer_callback));


  // rcl_ret_t rc3 = rclc_publisher_init_default(
  //   &u3_publisher, &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64), u3_topic);





  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, number_of_handles, &allocator));

  // Add handles to executor
  // Throttle subscriber
  RCCHECK(rclc_executor_add_subscription(&executor, &throttle_subscriber, &throttle_msg, &throttle_subscriber_callback, ON_NEW_DATA));

  // Brake subscriber
  RCCHECK(rclc_executor_add_subscription(&executor, &brake_subscriber, &brake_msg, &brake_subscriber_callback, ON_NEW_DATA));


  // RCCHECK(rclc_executor_add_subscription(&executor, &u1_publisher, &u1_msg, &throttle_subscriber_callback, ON_NEW_DATA));



  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));
}

/*
   Updates throttle output value to setpoint value

   Input:
   int setpoint = min_throttle_output-max_throttle_output
*/
void updateThrottle(int setpoint) {
  analogWrite(throttle_output_pin, setpoint);
}

/*
   Updates brakes to move towards setpoint value

   Input:
   int setpoint = min_brake_output-max_brake_output
*/
void updateBrakes(int setpoint) {
  // Rear brake
  rear_brake_position = analogRead(rear_brake_position_pin);
  current_rear_error = setpoint - rear_brake_position;

  if (current_rear_error > brake_tolerance) {
    // Error > Tolerance = Brake is not pressed far enough
    // Push brake forward
    setBrakeState(Brake::rear, BrakeState::push);
  }

  else if (current_rear_error < -brake_tolerance) {
    front_brake_position = analogRead(front_brake_position_pin);
    current_front_error = setpoint - front_brake_position;

    if (current_front_error > brake_tolerance) {
      // Error > Tolerance = Brake is not pressed far enough
      // Push brake forward
      setBrakeState(Brake::front, BrakeState::push);
    } else if (current_front_error < -brake_tolerance) {
      // Error < -Tolerance = Brake is pressed too far
      // Pull brake back
      setBrakeState(Brake::front, BrakeState::pull);
    } else {
      // Error < |Tolerance| = Brake is pressed to a position within the tolerance window
      // Do not move brake
      setBrakeState(Brake::front, BrakeState::stay);
    }
  }
}

/*
   Function to increase readability of brake actuation. Uses custom enum classes "Brake" and "BrakeState"
   which specifiy which brake to actuate, and how the brake should be actuated.

   Inputs:
   Brake brake = Brake::rear, Brake::front, Brake::both
   BrakeState brake_state = BrakeState::push, BrakeState::pull, BrakeState::stay
*/
void setBrakeState(Brake brake, BrakeState brake_state) {
  // Rear brake
  if (brake == Brake::rear || brake == Brake::both) {
    if (brake_state == BrakeState::push) {
      digitalWrite(rear_brake_backward_pin, LOW);
      digitalWrite(rear_brake_forward_pin, HIGH);
    } else if (brake_state == BrakeState::pull) {
      digitalWrite(rear_brake_forward_pin, LOW);
      digitalWrite(rear_brake_backward_pin, HIGH);
    } else if (brake_state == BrakeState::stay) {
      digitalWrite(rear_brake_forward_pin, LOW);
      digitalWrite(rear_brake_backward_pin, LOW);
    }
  }

  // Front brake
  if (brake == Brake::front || brake == Brake::both) {
    if (brake_state == BrakeState::push) {
      digitalWrite(front_brake_backward_pin, LOW);
      digitalWrite(front_brake_forward_pin, HIGH);
    } else if (brake_state == BrakeState::pull) {
      digitalWrite(front_brake_forward_pin, LOW);
      digitalWrite(front_brake_backward_pin, HIGH);
    } else if (brake_state == BrakeState::stay) {
      digitalWrite(front_brake_forward_pin, LOW);
      digitalWrite(front_brake_backward_pin, LOW);
    }
  }
}
