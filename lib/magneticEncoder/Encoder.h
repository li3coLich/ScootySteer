#ifndef ENCODER_h
#define ENCODER_h

#include "Arduino.h"
#include <AS5047P.h>



class Encoder {
  public:
    Encoder(uint8_t chipSelectPinNo = 5, uint32_t spiSpeed = 100000);
    void init();
    uint16_t ecd_getAngleRaw();
    float ecd_getMechanicalAngle();
    void ecd_update();
    float ecd_getAngle();
    float ecd_getVelocity(); 
  private:
    float prev_angle = 0;
    long prev_ts_angle = 0;
    float prev_angle_velocity = 0;
    long prev_ts_angle_velocity = 0;
    int32_t rotations = 0;
    int32_t full_rotations = 0;
    AS5047P as5047p;
};

#endif 