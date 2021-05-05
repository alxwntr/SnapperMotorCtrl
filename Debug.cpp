#include "Debug.h"

//Two info structs to hold data
snapper_msgs::pidInfo pidInfo[2];

//Array to hold the structs
snapper_msgs::ctrlDebug debugInfo;

void publish_debug( ros::Publisher& pub ) {
  debugInfo.data=pidInfo;
  debugInfo.data_length=2;
  pub.publish( &debugInfo );
}
