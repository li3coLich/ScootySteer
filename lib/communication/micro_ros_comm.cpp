#include "micro_ros_comm.h"


rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_publisher_t esp32_steer_publisher;
geometry_msgs__msg__Vector3 esp32_steer_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

float MicroRosComm::linear_vel = 0.0f;
float MicroRosComm::angular_vel = 0.0f;

extern float steer_speed;
extern MotorFOC motorFOC;
extern float currentSpeed;

#define LED_PIN 2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
  }
}

MicroRosComm::MicroRosComm(){
}

void MicroRosComm::init(){
    Serial.begin(115200);
    Serial.println("ROS Communication node started");
    
    set_microros_serial_transports(Serial);
    delay(3000);

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32", "", &support));
}

void MicroRosComm::publisher_start(){
    RCCHECK(rclc_publisher_init_default(
    &esp32_steer_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "/esp32_msg"));
}

void MicroRosComm::executors_start(){
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,&MicroRosComm::cmd_vel_callback, ON_NEW_DATA));

    Serial.println("Executors Started");
}
void MicroRosComm::subscriber_define(){
    RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));
}

void MicroRosComm::cmd_vel_callback(const void *msg_recv){
    const geometry_msgs__msg__Twist * recieved_data = (const geometry_msgs__msg__Twist *) msg_recv ;

    steer_speed = recieved_data->angular.z;
    
    // if (linear_vel != 0) {
    //     digitalWrite(2,HIGH);
    // }
    // srv_setDuty(steer, pwm);
    // bldc_setTargetSpeed(drivebldc, linear_vel);
}

void MicroRosComm::start_receiving_msgs(){
    esp32_steer_msg.x = motorFOC.motor_getAngle();
    esp32_steer_msg.y = currentSpeed;
    esp32_steer_msg.z = steer_speed;
    RCSOFTCHECK(rcl_publish(&esp32_steer_publisher, &esp32_steer_msg, NULL));
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    // bldc_setTargetSpeed(drivebldc, linear_vel);
}