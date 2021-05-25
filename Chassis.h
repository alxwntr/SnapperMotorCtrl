#ifndef CHASSIS_H
#define CHASSIS_H

#include <Arduino.h>
#include <snapper_msgs/setCarDimensions.h>
#include <sensor_msgs/JointState.h>

extern float wheelbase;
extern float wheelDia;
extern float gearboxRatio;

extern sensor_msgs::JointState rotations;

void set_dims (const snapper_msgs::setCarDimensions &dims);

void store_rotations (float angleIncrement[2], int wheels);

#endif
