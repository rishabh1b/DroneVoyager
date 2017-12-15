#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "ardrone_autonomy/Navdata.h"
#include "Eigen/Dense"
#include "Eigen/LU"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Char.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <algorithm>

#define KEYCODE_L 0x6C // lock slam mapping and save map
#define KEYCODE_U 0x75 // unlock mapping
#define KEYCODE_R 0x72 // reset map

#define KEYCODE_M 0x6D // disable control back to manual, disable setpoint.
#define KEYCODE_G 0x67 // get startframe and setpoint to hold place in startframe, enable control
#define KEYCODE_0 0x30 // set startframe to origin and setpoint to hold place where it is, enable control
#define KEYCODE_P 0x70 // follow path relative to startframe, enable control

/** parameters */

double euler_angle_max;
double control_vz_max;
double control_yawrate_max;
bool usehover; /** if usehover =1 uses the ardrone hover stabilization when near=1*/
double near_upper_threshold;
double near_lower_threshold;
double kpyaw;
double kdyaw;
double kpaltd;
double K1;
double K2;

/** DECLARES GLOBAL VARIABLES */

uint64_t timestamp; /** drone's timestamp buffer MUST BE RETENTIVE*/
double takeofftime;
geometry_msgs::Vector3 angular_velocity;
geometry_msgs::Twist poseRateSP; /** velocitys setpoints */
tf::StampedTransform transformPoseError; /** transform between /ardrone_base_link /poseSetPoint*/
int near; /* if distance to setpoint is small */
bool enablecontrol; /** to enable the control focus the terminal and press the C key on keyboard */

tf::StampedTransform transformCameraToRobot;
tf::Transform transformCameraToRobotUnStamped;


class GoThroughWindow {
private:
	ros::NodeHandle n_;
	ros::Publisher cmdpub_;
    ros::Publisher fakecmdpub_;
	ros::Subscriber navdataSub_ ;
	ros::Subscriber arTagSubscriber_;
	ros::Subscriber windowCentreSubscriber_;
	ros::Subscriber keySubscriber_;
	tf::StampedTransform predictedpose; /**  pose predicted by the slam node */
	tf::Transform startTransform; /** the tranform between /map and /startFrame, wich is the pose at the first second of flight */
	tf::Transform SPTransform; /** the tranform between /startFrame and /PoseSetPoint, witch is the set point related to the start frame */
	ros::Publisher errpub_;
	double wind_x_, wind_y_, wind_z_;
	bool centre_found;
	bool window_found_;
	std::string marker_frame;
    tf::Transform transformWindowCentre;
    tf::TransformListener listener;
    tf::TransformListener camera_to_base;
    // New Parameters
    int top_left_, top_right_, bottom_right_, bottom_left_;
    std::vector<int> tagids;
    bool first_time_four_tags;
    std::vector<int> top_lefts, bottom_lefts, bottom_rights, top_rights;
    int callbackARTagCounter;
    bool four_tags_located, tagsIdentified;
public:
	GoThroughWindow();
	void updatePose(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
	void updateError(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
	void getWindowCentre(const geometry_msgs::Point msg);
	void Control(const ardrone_autonomy::Navdata navdata);
	void keyCallBack(const std_msgs::Char key);
	void IdentifyTags(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
};