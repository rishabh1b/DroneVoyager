#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"

class OpenLoopControl {
public:
	OpenLoopControl(ros::NodeHandle nh);
	void setWaypoint(float x);
	void execute();
private:
	ros::NodeHandle nh_;
	ros::Publisher velPub_, takeOffPub_, landingPub_;
	geometry_msgs::Twist msg_;
	double timer;
};