#include <Arduino.h>

#include "MotorPID.h"
#include "Chassis.h"
#include "Debug.h"

/* This turns on errors for all implicit int<->float conversions and so
 * on. I think it is worth keeping these explicit, because the results
 * can be confusing otherwise. Note that by default the Arduino build
 * tools turn all warnings off (with -w) which will prevent this; the
 * option needs to be changed in the GUI. */
#pragma GCC diagnostic error "-Wconversion"

//-------------------------
//  PID variables
//-------------------------

//These are adjustable from the ROS interface via the setGains message.
static float Kp = 0.0f;
static float Ki = 1.0f;
static float Kd = 0.0f;

void
set_gains(float propG, float intG, float diffG)
{
  Kp = propG;
  Ki = intG;
  Kd = diffG;
}

//-------------------
// MotorController Class
//-------------------

void 
MotorController::setup_pins ()
{
  //Motor pins
  pinMode(motorA_, OUTPUT);
  pinMode(motorB_, OUTPUT);

  encoder_.setup_pins();
}

Direction 
MotorController::find_direction(float demand)
{
  if (demand > 0)
    return Forward;

  if (demand < 0)
    return Backward;

  return Stopped;
}

int
MotorController::find_pwm(float demand, float speed)
{
  float   error, pwm;
  
  pidInfo[motorNum].direction = (speed==0?0:speed>0?1:-1); //backwards = -1, stationary = 0, forwards = 1.

  //Calculate error and set PWM level:
  error       = demand - speed;
  errorSum_   += error * dT;
  errorSum_   = constrain(errorSum_, -255, 255);
  pwm         = Kp * error + Ki * errorSum_ + Kd * (error - lastError_) / dT;
  lastError_  = error;

  //Implement 'dead zone' to kill decay tail
  if (pwm > -15 && pwm < 15) pwm = 0;
  
  //Store values in debug struct
  pidInfo[motorNum].error = error;
  pidInfo[motorNum].speed = speed;
  pidInfo[motorNum].pwm = (int16_t)pwm;

  return int(pwm);
}

void
MotorController::write_to_pins(int pwm)
{
  if (pwm >= 0) {
    pwm = constrain(pwm, 0, 255);
    digitalWrite(motorA_, 0); //direction select pin = 0 => forwards
    analogWrite(motorB_, pwm);
  }
  else {
    pwm = -pwm;
    pwm = constrain(pwm, 0, 255);
    digitalWrite(motorA_, 1); //direction select pin = 1 => backwards
    analogWrite(motorB_, pwm);
  }
}

//PID loop:
void 
MotorController::process_pid (const geometry_msgs::Twist &twist)
{
  float twX   = float(twist.linear.x);
  float twTh  = float(twist.angular.z);

  //Demanded floor speed for this motor
  float demandFS  = twX + (RHS ? -1 : 1) * twTh * wheelbase / 2;
  //Demanded encoder speed for this motor
  float demandEnc = demandFS * gearboxRatio / (wheelDia/2); 
  // (Radians per sec for the encoder)
  float speed     = encoder_.speed();

  auto pwm  = find_pwm(demandEnc, speed);
  write_to_pins(pwm);
}
