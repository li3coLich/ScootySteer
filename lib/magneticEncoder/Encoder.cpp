#include "Encoder.h"

#define LED_PIN 13

Encoder::Encoder(uint8_t chipSelectPinNo, uint32_t spiSpeed) : as5047p(chipSelectPinNo, spiSpeed) {

}
void Encoder::init(){
	as5047p.initSPI();
}

uint16_t Encoder::ecd_getAngleRaw() {
	return as5047p.readAngleRaw();
}

float Encoder::ecd_getMechanicalAngle(){
	return ecd_getAngleRaw() * 2 * PI/16384.0f;
}

void Encoder::ecd_update() {
    float val = ecd_getMechanicalAngle();
    prev_ts_angle = micros();
    float delta_angle = val - prev_angle;
    if(abs(delta_angle) > (0.8f*2*PI) ) rotations += ( delta_angle > 0 ) ? -1 : 1; 
    prev_angle = val;
}

float Encoder::ecd_getAngle(){
    return (float)rotations * 2 * PI + prev_angle;
}


float Encoder::ecd_getVelocity() {
    float Ts = (prev_ts_angle - prev_ts_angle_velocity)*1e-6;
    if(Ts <= 0) Ts = 1e-3f;
    float vel = ( (float)(rotations - full_rotations)* 2 * PI + (prev_angle - prev_angle_velocity) ) / Ts;    
    prev_angle_velocity = prev_angle;
    full_rotations = rotations;
    prev_ts_angle_velocity = prev_ts_angle;
    return vel;
}

