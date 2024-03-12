#ifndef MICROROSCOMM_H
#define MICROROSCOMM_H

#include <Arduino.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>

#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/float32_multi_array.h>
#include "motorFOC.h"

class MicroRosComm{
    public:
        MicroRosComm();
        void init();
        void subscriber_define();
        static void cmd_vel_callback(const void *msg_recv);
        void start_receiving_msgs();
        void executors_start();
        void publisher_start();

        float get_linear_vel() const {
            return linear_vel;
        }

        float get_angular_vel() const {
            return angular_vel;
        }

    private:
        static float linear_vel;
        static float angular_vel;
};


#endif