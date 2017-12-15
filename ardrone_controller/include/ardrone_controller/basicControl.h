#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/Range.h"

class OpenLoopControl {
public:
	OpenLoopControl(ros::NodeHandle nh);
	void setWaypoint(float x);
	void execute();
	void execute2();
	void inPlaceYaw();
	void heightCallback(const sensor_msgs::Range r);
private:
	ros::NodeHandle nh_;
	ros::Publisher velPub_, takeOffPub_, landingPub_;
	ros::Subscriber heightSub_;
	geometry_msgs::Twist msg_;
	double timer, curr_height;
};