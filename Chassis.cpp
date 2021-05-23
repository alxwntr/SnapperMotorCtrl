#include <Arduino.h>

#include "Chassis.h"
#include "MotorArray.h"

float wheelbase = 0.230;
float wheelDia=0.084;
float gearboxRatio=89.94;

sensor_msgs::JointState rotations;

void
set_dims(const snapper_msgs::setCarDimensions &dims)
{
  wheelbase = dims.wheelBase;
  wheelDia = dims.wheelDia;
  gearboxRatio = dims.gearboxRatio;
}

void
setupJS()
{
  rotations.name_length = 2;
  char *id = "base_link";
  char *a[] = {"left_wheel_joint", "right_wheel_joint"};
  rotations.header.frame_id = id;
  rotations.name = a;
  rotations.position_length = 2;
}

void
store_rotations(float rots[2])
{
  for (int i=0; i++; i<2)
  {
    float rot_tot = rotations.position[i] + rots[0];
    if (rot_tot > PI)
    {
      rotations.position[i] = -2*PI + rot_tot; 
    }
    else if (rot_tot < -PI)
    {
      rotations.position[i] = 2*PI - rot_tot;
    }
    else rotations.position[i] = rot_tot;
  }
}
