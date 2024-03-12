#include <Arduino.h>
#include "Encoder.h"
#include "motorFOC.h"
#include "micro_ros_comm.h"

MotorFOC motorFOC;
MicroRosComm micro_ros_comm;
int count = 0;
bool scooterMode = true; //user mode
float steer_speed = 0.0f;
float currentSpeed = 0.0f;

void steerMotor(void *pvParameters) {
  for (;;) {
    if(scooterMode == true){
      // motorFOC.motor_setVirtualWall();
      // currentSpeed = motorFOC.motor_setAngularVelocity(10.0f);
      // currentSpeed = motorFOC.motor_setAngularVelocityWithVirtualWall(steer_speed);
    }
    else {
    }
    // Delay for 10ms
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void taskInit(){
  // srv_initial(steer);
  // srv_setDuty(steer, 15.0f);
  xTaskCreate(
    steerMotor,          // Task function
    "steerMotor",        // Name of the task
    4096,           // Stack size in words
    NULL,           // Task input parameter
    1,              // Priority of the task
    NULL);          // Task handle

  // xTaskCreate(
  //   readRearBrake,          // Task function
  //   "read rear break",        // Name of the task
  //   1024,           // Stack size in words
  //   NULL,           // Task input parameter
  //   1,              // Priority of the task
  //   NULL);          // Task handle
}

void setup() {

  Serial.begin(115200);
  motorFOC.motor_inital(12,6);
  motorFOC.motor_enable();
  motorFOC.motor_alignSensor();
  motorFOC.motor_initalPIDandFilter();
  motorFOC.motor_setMaxMinAngle(108*PI/180, -108*PI/180);
  Serial.println("...............");
  Serial.println(motorFOC.motor_getZeroElectricAngle());
  // taskInit();
  // micro_ros_comm.init();
  // micro_ros_comm.publisher_start();
  // micro_ros_comm.subscriber_define();
  // micro_ros_comm.executors_start();
}

void loop() {
  // float angle = as5047p.readAngle();
  // micro_ros_comm.start_receiving_msgs();
  // // Show the measure angle on the Serial Port
  // float angle = motorFOC.motor_getAngle();
  // motorFOC.motor_velocityOpenloop(5);
  float currentSpeed = motorFOC.motor_setAngularVelocity(6);
  // motorFOC.motor_enable();
  // currentSpeed = motorFOC.motor_setAngularVelocity(10.0f);
  count++;
  if(count % 100 == 0) {
    Serial.println(currentSpeed);
  }
  // // Serial.println(angle);
  // motorFOC.motor_setTorque(Kp*(6.28-angle)*180/PI,motorFOC.motor_getMechanicalAngle());   //位置闭环
}