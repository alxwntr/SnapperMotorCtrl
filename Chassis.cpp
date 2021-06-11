#include <Arduino.h>

#include "Chassis.h"
#include "MotorArray.h"

float wheelbase = 0.230;
float wheelDia = 0.084;
float gearboxRatio = 89.94;

void
set_dims(float WB, float WD, float GB)
{
  wheelbase = WB;
  wheelDia = WD;
  gearboxRatio = GB;
}

void
store_wheel_states(float angleIncrement[2], int wheels)
{
  for (int i=0; i<wheels; i++)
  {
    float new_posn = wheelStates.position[i] + angleIncrement[i];
    if (new_posn > PI)
    {
      wheelStates.position[i] = -2*PI + new_posn; 
    }
    else if (new_posn < -PI)
    {
      wheelStates.position[i] = 2*PI - new_posn;
    }
    else wheelStates.position[i] = new_posn;
  }
}
