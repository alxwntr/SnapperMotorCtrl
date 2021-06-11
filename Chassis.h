#ifndef CHASSIS_H
#define CHASSIS_H

#include <Arduino.h>
#include <snapper_msgs/setCarDimensions.h>
#include <sensor_msgs/JointState.h>

extern float wheelbase;
extern float wheelDia;
extern float gearboxRatio;

extern sensor_msgs::JointState wheelStates;

void set_dims (float WB, float WD, float GB);

void store_wheel_states (float angleIncrement[2], int wheels);

#endif
