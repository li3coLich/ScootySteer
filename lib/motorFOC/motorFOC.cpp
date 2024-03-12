#include "motorFOC.h"

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

MotorFOC::MotorFOC(uint8_t _pwmA, uint8_t _pwmB, uint8_t _pwmC, uint8_t _en, uint8_t _PP, int8_t _DIR){
    pwmA = _pwmA;
    pwmB = _pwmB;
    pwmC = _pwmC;
    EN = _en;
    PP = _PP;
    DIR = _DIR;
};

float MotorFOC::_normAngle(float angle){
    float output = fmod(angle, 2 * PI);   
  return output >= 0 ? output : (output + 2 * PI);  
}

void MotorFOC::motor_setTorque(float Uq, float angle_el) {
  Uq = _constrain(Uq,-voltage_power_supply / 2,voltage_power_supply / 2);
  float Ud = 0;
  angle_el = _normAngle(angle_el);
  Ualpha =  -Uq * sin(angle_el); 
  Ubeta =   Uq * cos(angle_el); 
  UA = Ualpha + voltage_power_supply / 2;
  UB = (sqrt(3) * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
  UC = (- Ualpha - sqrt(3) * Ubeta) / 2 + voltage_power_supply / 2;
  setPwm(UA,UB,UC);
}

float MotorFOC::motor_getMechanicalAngle(){
  return  _normAngle((float)(DIR *  PP) * encoder.ecd_getMechanicalAngle() - zero_electric_angle);
}

void MotorFOC::setPwm(float Ua, float Ub, float Uc) {

    Ua = _constrain(Ua, 0.0f, voltage_limit);
    Ub = _constrain(Ub, 0.0f, voltage_limit);
    Uc = _constrain(Uc, 0.0f, voltage_limit);

    float dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
    float dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
    float dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

    ledcWrite(0, dc_a*255);
    ledcWrite(1, dc_b*255);
    ledcWrite(2, dc_c*255);
}

void MotorFOC::motor_inital(float voltage_supply, float _voltage_limit){
    voltage_limit = _voltage_limit;
    voltage_power_supply = voltage_supply;
    pinMode(pwmA, OUTPUT);
    pinMode(pwmB, OUTPUT);
    pinMode(pwmC, OUTPUT);
    pinMode(EN, OUTPUT);
    digitalWrite(EN, LOW);   // enable
    ledcSetup(0, 30000, 8);  
    ledcSetup(1, 30000, 8);  
    ledcSetup(2, 30000, 8);  
    ledcAttachPin(pwmA, 0);
    ledcAttachPin(pwmB, 1);
    ledcAttachPin(pwmC, 2);
    Serial.println("PWM and EN ready.");

    encoder.init();
    Serial.println("encoder ready.");

    pid_velocity = PIDController(2, 0, 0, 10000, voltage_power_supply/2);
}

void MotorFOC::motor_alignSensor() {
    motor_setTorque(3, 1.5 * PI);
    delay(1000);
    encoder.ecd_update();
    zero_electric_angle = motor_getMechanicalAngle();
    motor_setTorque(0, 1.5 * PI);
    Serial.print("0 degree:");
    Serial.println(zero_electric_angle);
}

void MotorFOC::motor_initalPIDandFilter(){
    lpf = LowPassFilter(0.05);
    pid_velocity = PIDController(0.005, 0.1, 0, 10000, voltage_power_supply/2);
    pid_angle = PIDController(2, 0, 0, 10000, 100);
}

float MotorFOC::motor_getAngle()
{
    encoder.ecd_update();
    return encoder.ecd_getAngle();
}

void MotorFOC::motor_enable() {
  digitalWrite(EN, HIGH);
}

void MotorFOC::motor_disable() {
  digitalWrite(EN, LOW);
}


void MotorFOC::motor_setVelocityPID(float P,float I,float D,float ramp, float limit)   //M0角度环PID设置
{
    pid_velocity.P = P;
    pid_velocity.I = I;
    pid_velocity.D = D;
    pid_velocity.limit = limit;
    pid_velocity.output_ramp = ramp;
}

void MotorFOC::motor_setAnglePID(float P,float I,float D,float ramp, float limit)   //M0角度环PID设置
{
    pid_angle.P = P;
    pid_angle.I = I;
    pid_angle.D = D;
    pid_angle.limit = limit;
    pid_angle.output_ramp = ramp;
}

float MotorFOC::motor_velocityUpdate()
{
    encoder.ecd_update();
    float ecd_velocity = encoder.ecd_getVelocity();
    float filtered_velocity = lpf(DIR*ecd_velocity, 1000);
    return filtered_velocity;
}

float MotorFOC::motor_getZeroElectricAngle(){
    return zero_electric_angle;
}


float MotorFOC::_openLoopElectricalAngle(float shaft_angle, int pole_pairs) {
  return (shaft_angle * pole_pairs);
}

float MotorFOC::motor_velocityOpenloop(float target_velocity){
  unsigned long now_us = micros(); 
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
  open_loop_shaft_angle = _normAngle(open_loop_shaft_angle + target_velocity*Ts);
  float Uq = voltage_power_supply/3;
  motor_setTorque(Uq, _openLoopElectricalAngle(open_loop_shaft_angle, PP));
  open_loop_timestamp = now_us;
  return Uq;
}

float MotorFOC::motor_getVelocityPIDError(float error){
    return pid_velocity(error);
}


float MotorFOC::motor_setAngularVelocity(float target_velocity){
    float vel = motor_velocityUpdate();
    float error = motor_getVelocityPIDError(target_velocity - vel);
    motor_setTorque(error * 180/ PI, motor_getMechanicalAngle());
    return vel;
}

float MotorFOC::motor_setMotorTorque(float target_torque){
    motor_setTorque(target_torque, motor_getMechanicalAngle());
}

void MotorFOC::motor_setMaxMinAngle(float _maxAngle, float _minAngle){
  maxAngle = _maxAngle;
  minAngle = _minAngle;
}

float MotorFOC::motor_setAngularVelocityWithVirtualWall(float target_velocity){
  float current_velocity = motor_velocityUpdate();
  float current_angle = motor_getAngle();
  float kp = 4;
  float kd = 10;
  float uq = 0;
  if (motor_getAngle() > maxAngle) {
    if(target_velocity > 0){
      float error = motor_getVelocityPIDError(target_velocity - current_velocity);
      motor_setTorque(error * 180/ PI, motor_getMechanicalAngle());
      return current_velocity;
    } else{
      if(current_velocity > 0){
        current_velocity = 0;
      }
      uq = 0;
      // uq = kp * (current_angle - maxAngle) -kd * current_velocity; 
      motor_setTorque(uq, motor_getMechanicalAngle());
      return current_velocity;
    }
  } else if (motor_getAngle() < minAngle) {
    if(target_velocity < 0){
      float error = motor_getVelocityPIDError(target_velocity - current_velocity);
      motor_setTorque(error * 180/ PI, motor_getMechanicalAngle());
      return current_velocity;
    } else{
      if(current_velocity<0){
        current_velocity = 0;
      }
      uq = 0;
      // uq = kp * (current_angle - minAngle) -kd * current_velocity;
      motor_setTorque(uq, motor_getMechanicalAngle());
      return current_velocity;
    }
  } else {
    float error = motor_getVelocityPIDError(target_velocity - current_velocity);
    motor_setTorque(error * 180/ PI, motor_getMechanicalAngle());
    return current_velocity;
  }
}

// void MotorFOC::motor_setVirtualWall(){
//   float current_velocity = motor_velocityUpdate();
//   float current_angle = motor_getAngle();
//   float kp = 3;
//   float kd = 1;
//   if (motor_getAngle() > maxAngle) {
//     if(current_velocity>0){
//       current_velocity = 0;
//     }
//     float uq = kp * (current_angle - maxAngle) -kd * current_velocity; //kp * (current_angle - maxAngle) + 
//     Serial.print(current_velocity); //+ kd * current_velocity;
//     motor_setTorque(uq, motor_getMechanicalAngle());
//   } else if (motor_getAngle() < minAngle) {
//     if(current_velocity<0){
//       current_velocity = 0;
//     }
//     float uq = kp * (current_angle - minAngle) -kd * current_velocity;  //kp * (current_angle - minAngle) +  //- kd * current_velocity;
//     Serial.print(current_velocity);
//     motor_setTorque(uq, motor_getMechanicalAngle());
//   } else {
//     motor_setTorque(0, motor_getMechanicalAngle());
//   }
// }