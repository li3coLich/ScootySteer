#ifndef MOTORFOC_H
#define MOTORFOC_H

#include "Arduino.h"
#include "Encoder.h"
#include "lowpass_filter.h"
#include "pid.h"


#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
class MotorFOC{

    public:
        MotorFOC(uint8_t pwmA = 13, uint8_t pwmB = 12, uint8_t pwmC = 14, uint8_t en = 27,
                uint8_t PP = 14, int8_t DIR = -1);
        void motor_inital(float voltage_supply, float _voltage_limit);
        void motor_initalPIDandFilter();
        void motor_enable();
        void motor_disable();
        void motor_alignSensor();
        void motor_setMaxMinAngle(float _maxAngle, float _minAngle);
        float motor_setAngularVelocityWithVirtualWall(float target_velocity);
        void motor_setTorque(float Uq,float angle_el);
        float motor_getAngle();
        float motor_getMechanicalAngle();
        void motor_setVelocityPID(float P,float I,float D,float ramp, float limit);
        void motor_setAnglePID(float P,float I,float D,float ramp, float limit);
        float motor_getVelocityPIDError(float error);
        float motor_velocityUpdate();
        float motor_getZeroElectricAngle();
        float motor_velocityOpenloop(float target_velocity);
        float motor_setAngularVelocity(float target_velocity);
        float motor_setMotorTorque(float target_torque);
        void motor_setVirtualWall();

    private:
        uint8_t PP;
        int8_t DIR;
        uint8_t pwmA, pwmB, pwmC, EN;
        Encoder encoder;
        LowPassFilter lpf = LowPassFilter(0.01);
        PIDController pid_velocity = PIDController(2, 0, 0, 10000, voltage_power_supply/2);
        PIDController pid_angle = PIDController(2, 0, 0, 10000, 100);
        float voltage_supply;
        float voltage_limit;
        float voltage_power_supply;
        float Ualpha, Ubeta = 0;
        float UA = 0, UB = 0, UC = 0;
        float zero_electric_angle = 0;
        float _normAngle(float angle);
        float _openLoopElectricalAngle(float shaft_angle, int pole_pairs);
        void setPwm(float UA, float UB, float UC);
        float open_loop_timestamp = 0;
        float open_loop_shaft_angle = 0;
        float maxAngle = 0;
        float minAngle = 0;
        float virtual_wall_prev_velocity = 0;
        float virtual_wall_prev_angle = 0;
        float virtual_wall__prev_timestamp = 0;
};


#endif