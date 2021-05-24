#include <ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <snapper_msgs/pidInfo.h>
#include <snapper_msgs/ctrlDebug.h>
#include <snapper_msgs/setGains.h>
#include <snapper_msgs/setCarDimensions.h>
#include <sensor_msgs/JointState.h>

#include "DeadReckoning.h"
#include "MotorArray.h"
#include "MotorPID.h"
#include "Debug.h"
#include "Chassis.h"

//To start communication, use:
//rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=1000000 (remember #define USE_USBCON 1)
//rosrun rosserial_python serial_node.py /dev/ttyAMA0  _baud:=1000000 (on pi3 serial pins)

//-------------------------
//  ROS stuff
//-------------------------

ros::NodeHandle nh;

geometry_msgs::Twist confirm;

const int linDmdMax = 2;
const int angDmdMax = 6;

//Publishers and subscriber:
ros::Publisher p1("demand_confirm", &confirm);
ros::Publisher p2("debug", &debugInfo);
ros::Publisher p3("joint_states", &rotations);

void confirmCallback(const geometry_msgs::Twist& msg)
{
  confirm.linear.x = constrain (msg.linear.x, -linDmdMax, linDmdMax);
  confirm.angular.z = constrain (msg.angular.z, -angDmdMax, angDmdMax);
  p1.publish( &confirm );
}

void gainSetCallback(const snapper_msgs::setGains& gainMsg)
{
  set_gains (gainMsg);
}

void gainCarDimsCallback(const snapper_msgs::setCarDimensions& dimsMsg)
{
  set_dims (dimsMsg);
}

ros::Subscriber<geometry_msgs::Twist> s1("cmd_vel", &confirmCallback);
ros::Subscriber<snapper_msgs::setGains> s2("set_gain", &gainSetCallback);
ros::Subscriber<snapper_msgs::setCarDimensions> s3("set_dims", &gainCarDimsCallback);

//  tf variables:
geometry_msgs::TransformStamped t;
char base_link[] = "/base_link";
char odom[] = "/odom";
//JS variabes:
sensor_msgs::JointState rotations;
static char *names[] = {"left_wheel_joint", "right_wheel_joint"};
static float rots[2] = {0, 0};
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
  nh.advertise(p1);
  nh.advertise(p2);
  nh.advertise(p3);
  nh.subscribe(s1);
  nh.subscribe(s2);
  nh.subscribe(s3);
  broadcaster.init(nh);

  setupJS();
}

void
setupJS()
{
  rotations.header.frame_id = base_link;
  rotations.name_length = 2;
  rotations.name = names;
  rotations.position_length = 2;
  rotations.position = rots;
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
  
  rotations.header.stamp = nh.now();

  p3.publish(&rotations);
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
    publish_debug( p2 );
  }

  nh.spinOnce();

  wait = micros() - mainLoopTime;
  wait = (1000000 * dT) - wait;
  delayMicroseconds(wait);
}
