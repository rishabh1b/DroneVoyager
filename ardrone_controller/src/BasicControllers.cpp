#include "ardrone_controller/basicControl.h"

OpenLoopControl::OpenLoopControl(ros::NodeHandle nh) {
	this->nh_ = nh;
	velPub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
	takeOffPub_ = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
	landingPub_ = nh.advertise<std_msgs::Empty>("/ardrone/land", 1);
	timer = 0;
}

void OpenLoopControl::execute() {
   std_msgs::Empty emptyMsg;
   takeOffPub_.publish(emptyMsg);

   while (takeOffPub_.isLatched()) {
   	// Kill Time
   	ROS_INFO(" Waiting for Takeoff");
   }

   double t = ros::Time::now().toSec();
   ros::Rate loop_rate(60);
   while (ros::ok() && ros::Time::now().toSec() - t < timer) {
          velPub_.publish(msg_);

   		 loop_rate.sleep();
	}
  landingPub_.publish(emptyMsg); 
}

void OpenLoopControl::setWaypoint (float x) {
	float max_velocity = 1;
    float min_velocity = -1;

    timer = x / max_velocity;
    if (x < 0) {
    	msg_.linear.x = min_velocity;
    } else {
    	msg_.linear.x = max_velocity;
    }
}