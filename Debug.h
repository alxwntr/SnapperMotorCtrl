#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
#include <ros.h>
#include <snapper_msgs/pidInfo.h>
#include <snapper_msgs/ctrlDebug.h>

extern snapper_msgs::pidInfo pidInfo[2];
//int16 pwm
//float32 speed
//float32 error
//int8 direction

extern snapper_msgs::ctrlDebug debugInfo;

void publish_debug( ros::Publisher& pub );

#endif
