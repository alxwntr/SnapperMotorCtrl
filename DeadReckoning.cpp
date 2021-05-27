#include <Arduino.h>

#include "DeadReckoning.h"
#include "MotorArray.h"
#include "Chassis.h"

// Odometer object:
Odometer odometer;

//Array to carry joint rotations
float angleIncrement[2];

//-------------------------
//  Movement functions
//-------------------------

void
Odometer::calculate_moves()
{
  float runningTotal = 0.0;
  float angTotal = 0.0;
  float fwdDist = 0.0;
  float dTheta = 0.0;
  

  for (auto &m : motors)
  {
    float rads = m.angle();
    auto wheelDist = (wheelDia/2) * rads;
    runningTotal += wheelDist;
    //Pos anti-clockwise, so add right, sub left
    angTotal += (m.RHS ? 1 : -1) * wheelDist; //Angle was the wrong way round! Left turn should be +ve.
    int idx = m.RHS;
    angleIncrement[idx] = rads; //Becomes [left, right]
  }
  fwdDist = runningTotal / motors.size();

  //Send wheel rotations to joint state store (TWO WHEELS)
  store_wheel_states(angleIncrement, 2);

  dTheta = angTotal / wheelbase;
  x += fwdDist * cos(theta + dTheta / 2);
  y += fwdDist * sin(theta + dTheta / 2);
  theta += dTheta;
}
