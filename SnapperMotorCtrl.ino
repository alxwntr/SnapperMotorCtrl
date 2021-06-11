#include <ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <snapper_msgs/pidInfo.h>
#include <snapper_msgs/ctrlDebug.h>
#include <snapper_msgs/SetGains.h>
#include <snapper_msgs/SetCarDimensions.h>
#include <sensor_msgs/JointState.h>
#include <rosserial_arduino/Test.h>

#include "DeadReckoning.h"
#include "MotorArray.h"
#include "MotorPID.h"
#include "Debug.h"
#include "Chassis.h"

//To start communication, use:
//rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=1000000 (on USB port, remember #define USE_USBCON 1)
//rosrun rosserial_python serial_node.py /dev/ttyAMA0  _baud:=1000000 (on pi3 serial pins)

//-------------------------
//  ROS stuff
//-------------------------

ros::NodeHandle nh;

geometry_msgs::Twist confirm;

const int linDmdMax = 2;
const int angDmdMax = 6;

//Publishers and subscriber:
ros::Publisher debug_pub("debug", &debugInfo);
ros::Publisher JS_pub("joint_states", &wheelStates);

void demandCallback(const geometry_msgs::Twist& msg)
{
  confirm.linear.x = constrain (msg.linear.x, -linDmdMax, linDmdMax);
  confirm.angular.z = constrain (msg.angular.z, -angDmdMax, angDmdMax);
}

using snapper_msgs::SetGains;
void callback(const SetGains::Request & req, SetGains::Response & res)
{
  set_gains (req.Kp, req.Ki, req.Kd);
  res.confirmation = "Gains set as requested. Be wary of instabilities.";
}

using snapper_msgs::SetCarDimensions;
void callback(const SetCarDimensions::Request & req, SetCarDimensions::Response & res)
{
  set_dims (req.wheelBase, req.wheelDia, req.gearboxRatio);
  res.confirmation = "Car dimensions set as requested";
}

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", &demandCallback);
ros::ServiceServer<SetGains::Request, SetGains::Response> gains_server("set_gains_srv",&callback);
ros::ServiceServer<SetCarDimensions::Request, SetCarDimensions::Response> dims_server("set_dims_srv",&callback);

//  tf variables:
geometry_msgs::TransformStamped t;
char base_link[] = "/base_link";
char odom[] = "/odom";
//JS variabes:
sensor_msgs::JointState wheelStates;
static char *names[] = {"left_wheel_joint", "right_wheel_joint"};
static float wheelAngles[2] = {0, 0};
tf::TransformBroadcaster broadcaster;

//-------------------------
//  Loop handling variables
//-------------------------

unsigned long mainLoopTime = 0;
unsigned long PIDLoopTime = 0;
unsigned long timeTemp = 0;
unsigned long wait = 0;
const int tfRateDivisor = 5;
int loopCount = 0;

//-------------------------
//  Functions
//-------------------------

void setup()
{
  setup_pins();

  nh.getHardware()->setBaud(1000000);
  nh.initNode();
  nh.advertise(debug_pub);
  nh.advertise(JS_pub);
  nh.subscribe(cmd_sub);
  nh.advertiseService(gains_server);
  nh.advertiseService(dims_server);
  broadcaster.init(nh);

  setupJS();
}

void
setupJS()
{
  wheelStates.header.frame_id = base_link;
  wheelStates.name_length = 2;
  wheelStates.name = names;
  wheelStates.position_length = 2;
  wheelStates.position = wheelAngles;
}

void publish_tf()
{
  // tf odom->base_link
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  
  t.transform.translation.x = odometer.x;
  t.transform.translation.y = odometer.y;
  
  t.transform.rotation = tf::createQuaternionFromYaw(odometer.theta);
  t.header.stamp = nh.now();

  broadcaster.sendTransform(t);
}

void publish_joint_states()
{
  
  wheelStates.header.stamp = nh.now();

  JS_pub.publish(&wheelStates);
}

//-------------------------
//  Main loop
//-------------------------

void loop()
{
  mainLoopTime = micros();
  loopCount += 1;

  for (auto &m: motors) {
    m.process_pid(confirm);
  }

  if (loopCount == tfRateDivisor)
  {
    odometer.calculate_moves();
    
    loopCount = 0;
    
    publish_tf();
    publish_joint_states();
    publish_debug( debug_pub );
  }

  nh.spinOnce();

  wait = micros() - mainLoopTime;
  wait = (1000000 * dT) - wait;
  delayMicroseconds(wait);
}
