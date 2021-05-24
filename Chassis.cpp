#include <Arduino.h>

#include "Chassis.h"
#include "MotorArray.h"

float wheelbase = 0.230;
float wheelDia=0.084;
float gearboxRatio=89.94;

void
set_dims(const snapper_msgs::setCarDimensions &dims)
{
  wheelbase = dims.wheelBase;
  wheelDia = dims.wheelDia;
  gearboxRatio = dims.gearboxRatio;
}

void
store_rotations(float rots[2], int wheels)
{
  for (int i=0; i<wheels; i++)
  {
    float rot_tot = rotations.position[i] + rots[i];
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
