#include "ardrone_controller/goThroughWindow.h"

#define tolerance 10

GoThroughWindow::GoThroughWindow() {
	errpub_ = n_.advertise<geometry_msgs::Twist>("/controlerror", 1);
	cmdpub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	fakecmdpub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel_fake", 1);
	windowCentreSubscriber_ = n_.subscribe("/window_cent", 1, &GoThroughWindow::getWindowCentre, this);
	// keySub_ = n_.subscribe("/keyinput", 1, &SubscribeAndPublish::keyCallBack, this);
	// poseRateSetPointSub_ = n_.subscribe("poseRateSetPoint", 1, &SubscribeAndPublish::poseRateSPCallback, this);
	navdataSub_ = n_.subscribe("ardrone/navdata", 1, &GoThroughWindow::Control, this);
	arTagSubscriber_ = n_.subscribe("ar_pose_marker", 10, &GoThroughWindow::updateError, this);
	keySubscriber_ = n_.subscribe("/keyinput", 10, &GoThroughWindow::keyCallBack, this);
	centre_found = false;
	enablecontrol = false;
	// Get the Static Transform
	try
	{
		listener.waitForTransform("ardrone_base_link", "ardrone_base_frontcam", ros::Time(0), ros::Duration(10.0) );
		listener.lookupTransform("ardrone_base_link", "ardrone_base_frontcam", ros::Time(0), transformCameraToRobot); /** gets the last published transformation */
	}
					
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());  /** handle errors */
	}
	transformCameraToRobotUnStamped.setRotation(transformCameraToRobot.getRotation());
	transformCameraToRobotUnStamped.setOrigin(transformCameraToRobot.getOrigin());
	first_time_four_tags = true;
	callbackARTagCounter = 0;
	four_tags_located = false;
	tagsIdentified = false;
	isFirstErrorCallback = true;
	targetPublished = false;
    vel_max = 0.1;
	dist_max = 3.5;
	yaw_threshold = 5;
	z_threshold = 0.1;
	y_threshold = 0.1;

}
void GoThroughWindow::Control(const ardrone_autonomy::Navdata navdata) {
/** GET TRANSFORM */
	//Populate transformPoseError here
	ROS_INFO("In new control");
	if (!targetPublished)
		return;
	try
	{
		listener.waitForTransform("/ardrone_base_link", "target", ros::Time(0), ros::Duration(10.0) );
		listener.lookupTransform("/ardrone_base_link", "target", ros::Time(0), transformPoseError); /** gets the last published transformation */
	}
		
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());  /** handle errors */
	}
	
	tf::Quaternion quat = transformPoseError.getRotation();
	tf::Vector3 trans = transformPoseError.getOrigin();


	tf::Transform currentPoseError;
	currentPoseError.setRotation(quat);
	currentPoseError.setOrigin(trans);

	double yawErr = acos(trans.getX() / (sqrt(std::pow(trans.getX(),2) + std::pow(trans.getY(),2)  + std::pow(trans.getZ(),2) ))) * 180 / 3.146;
	
	double altitudeErr;
	altitudeErr = currentPoseError.getOrigin().z();
	
	Eigen::Vector2d xyErr;
	xyErr << currentPoseError.getOrigin().x() , currentPoseError.getOrigin().y() ;		
	geometry_msgs::Twist error;
	
	error.linear.x=xyErr(0);
	error.linear.y=xyErr(1);
	error.linear.z=altitudeErr;
	error.angular.x=0;
	error.angular.y=0;
	error.angular.z=yawErr; 
	
	errpub_.publish(error); 
							
	
	
	/** CONTROLLER */
	
	geometry_msgs::Twist controlsig;
	
	
	/** ALTITUDE control */
	if (std::abs(yawErr) > yaw_threshold)
		controlsig.angular.z = yawErr * 0.1 / 60; // TODO: put a variable to this								 
	else if (std::abs(altitudeErr) > z_threshold) 
		controlsig.linear.z = vel_max * altitudeErr / 0.5; // TODO: put a variable for this
	else if (std::abs(y) > y_threshold) 
		controlsig.linear.y = vel_max * xyErr(1) / dist_max;
	else
		controlsig.linear.x = vel_max * xyErr(0) / dist_max;

	

	/** HORIZONTAL POSITION control */
	
	/** if distace is less than 15cm than it is near and will use hovermode.
	 * if it is in hover mode (near=1), and distance increses to more than 30cm, then leave hovermode */
	 
	/*if (sqrt(xyErr(1)*xyErr(1)+xyErr(0)*xyErr(0))<near_lower_threshold)
	{
		near = 1;
	}
	else
	{
		if (sqrt(xyErr(1)*xyErr(1)+xyErr(0)*xyErr(0))>near_upper_threshold)
		{
			near=0;
		}
	}*/
	
	
	/*if (near) // For later
	{
		// Get into Hover mode
		controlsig.linear.x = 0;
		controlsig.linear.y = 0;
		controlsig.angular.x = 0;   
		controlsig.angular.y = 0;	
	}*/

	// else /** if it is not in hover mode calculates all control signals using PD law */
	//{						
		
		/** control signals not used, but must be != 0 to stay out of hover mode */
		controlsig.angular.x = 0 ;   
		controlsig.angular.y = 0 ;		
	//}


	
	/** limit the control signals (talvez não seja necessário) */
	

	if (controlsig.angular.z > 1)
	{
		controlsig.angular.z =1;
	}
	if (controlsig.angular.z < -1)
	{
		controlsig.angular.z =-1;
	}
						
	if (controlsig.linear.x > 1)
	{
		controlsig.linear.x =1;
	}
	if (controlsig.linear.x < -1)
	{
		controlsig.linear.x =-1;
	}
	
	if (controlsig.linear.y > 1)
	{
		controlsig.linear.y =1;
	}
	if (controlsig.linear.y < -1)
	{
		controlsig.linear.y =-1;
	}
	
	if (controlsig.linear.z > 1)
	{
		controlsig.linear.z =1;
	}
	if (controlsig.linear.z < -1)
	{
		controlsig.linear.z =-1;
	}
	
	
	/** PUBLISH CONTROL SIGNALS */
	// False values
	fakecmdpub_.publish(controlsig);
	
	if (enablecontrol)
	{
		 cmdpub_.publish(controlsig); 
	}
	
		
		/** end of the control step */
} /** end of the callback function for the class SubscriveAndPublish*/

void GoThroughWindow::getWindowCentre(const geometry_msgs::Point msg) {
	if (!centre_found && window_found_) {
		ROS_INFO("Populating window values");
		wind_x_ = msg.x;
		wind_y_ = msg.y;
		wind_z_ = msg.z;
	}
}

void GoThroughWindow::updateError(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg){
	 // ROS_INFO("Came in Alvar message callback");	
	static tf::TransformBroadcaster br;
	std::cout << "Markers size: " << msg->markers.size() << std::endl;
	 if (msg->markers.size() != 4 && first_time_four_tags)
	 	return;

	 if (msg->markers.size() == 0)
	 	return;

	ROS_INFO("THIS IS WINDOW 3!!");
    if (!first_time_four_tags) {
    	oldx = x;
        oldy = y;
        oldz = z;
		int minindex = 0;
    	int minindex2 = 0;
    	float min_x = msg->markers[0].pose.pose.position.x;
    	float min_x2 = 100000;
    	for (int i = 1; i < msg->markers.size(); i++) {
    		if (msg->markers[i].pose.pose.position.x < min_x) {
    			min_x2 = min_x;
    			minindex2 = minindex;
    			min_x = msg->markers[i].pose.pose.position.x;
    			minindex = i;
    		}
    		else if (msg->markers[i].pose.pose.position.x < min_x2) {
				min_x2 = msg->markers[i].pose.pose.position.x;
        		minindex2 = i;
    		}
    	}
    	if(msg->markers[minindex2].pose.pose.position.y < msg->markers[minindex].pose.pose.position.y) {
    		minindex = minindex2;
    	}
	    x = msg->markers[minindex].pose.pose.position.x + wind_x_;
	    y = msg->markers[minindex].pose.pose.position.y + wind_y_;

	    float total_z;
	    int j;
	    for (j = 0; j < msg->markers.size(); j++){
	    	total_z += msg->markers[j].pose.pose.position.z;
	    }
	    z = total_z / j;

	    if (!isFirstErrorCallback && (std::abs(x-oldx)>tolerance || std::abs(y-oldy)>tolerance)) {
	    	x = oldx;
	    	y = oldy;
	    	z = oldz;
	    	// targetPublished = false;
	    }
        
        isFirstErrorCallback = false;

	    transformWindowCentre.setOrigin( tf::Vector3(x, y, z) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transformWindowCentre.setRotation(q);
        br.sendTransform(tf::StampedTransform(transformWindowCentre, ros::Time::now(), "/ardrone_base_frontcam", "target"));
        targetPublished = true;
    } else { // For Inidicating to stop in place hover
    	first_time_four_tags = false;
    	window_found_ = true;
		int minindex = 0;
    	int minindex2 = 0;
    	float min_x = msg->markers[0].pose.pose.position.x;
    	float min_x2 = 100000;
    	for (int i = 1; i < msg->markers.size(); i++) {
    		if (msg->markers[i].pose.pose.position.x < min_x) {
    			min_x2 = min_x;
    			minindex2 = minindex;
    			min_x = msg->markers[i].pose.pose.position.x;
    			minindex = i;
    		}
    		else if (msg->markers[i].pose.pose.position.x < min_x2) {
				min_x2 = msg->markers[i].pose.pose.position.x;
        		minindex2 = i;
    		}
    	}
    	if(msg->markers[minindex2].pose.pose.position.y < msg->markers[minindex].pose.pose.position.y) {
    		minindex = minindex2;
    	}
	    oldx = msg->markers[minindex].pose.pose.position.x + wind_x_;
	    oldy = msg->markers[minindex].pose.pose.position.y + wind_y_;

	    float total_z;
	    int j;
	    for (j = 0; j < msg->markers.size(); j++){
	    	total_z += msg->markers[j].pose.pose.position.z;
	    }
	    oldz = total_z / j;
    }

}

void GoThroughWindow::keyCallBack(const std_msgs::Char key)
	{
		switch(key.data)
		{
			case KEYCODE_G:
				enablecontrol =true;
				ROS_INFO("Control enabled. Drone holding place");
				break;   
				
			case KEYCODE_0:
				enablecontrol =true;
				ROS_INFO("Control enabled. Drone holding place");
				break;  

			case KEYCODE_P:
				enablecontrol =true;
				ROS_INFO("Control enabled. Drone following path");
				break;  

			case KEYCODE_M:
				enablecontrol =false;
				ROS_INFO("Control disabled. Drone back in manual");
				break;   
		}
	}

int main(int argc, char **argv) {
	ros::init(argc, argv, "pass_window");	
	
	ROS_INFO("ArDroneControl: position_control started");
	
	/** get parameters */
	ros::NodeHandle private_node_handle("~");
	
	private_node_handle.param<double>("euler_angle_max", euler_angle_max, 0.2);
	ROS_INFO("ArDroneControl - Position_control: euler_angle_max = %f", euler_angle_max);
	private_node_handle.param<double>("control_vz_max", control_vz_max, 700);
	ROS_INFO("ArDroneControl - Position_control: control_vz_max = %f", control_vz_max);
	private_node_handle.param<double>("control_yawrate_max", control_yawrate_max, 1.75);
	ROS_INFO("ArDroneControl - Position_control: control_yawrate_max = %f", control_yawrate_max);
	private_node_handle.param<bool>("usehover", usehover, false);
	ROS_INFO("ArDroneControl - Position_control: usehover = %d", usehover);	
	private_node_handle.param<double>("near_upper_threshold", near_upper_threshold, 0.6);
	ROS_INFO("ArDroneControl - Position_control: near_upper_threshold = %f", near_upper_threshold);
	private_node_handle.param<double>("near_lower_threshold", near_lower_threshold, 0.3);
	ROS_INFO("ArDroneControl - Position_control: near_lower_threshold = %f", near_lower_threshold);
	
	private_node_handle.param<double>("kpyaw", kpyaw, 1.5);
	ROS_INFO("ArDroneControl - Position_control: kpyaw = %f", kpyaw);
	private_node_handle.param<double>("kdyaw", kdyaw, 0.5);
	ROS_INFO("ArDroneControl - Position_control: kdyaw = %f", kdyaw);
	private_node_handle.param<double>("kpaltd", kpaltd, 0.45);
	ROS_INFO("ArDroneControl - Position_control: kpaltd = %f", kpaltd);
	private_node_handle.param<double>("K1", K1, 0.07);
	ROS_INFO("ArDroneControl - Position_control: K1 = %f", K1);
	private_node_handle.param<double>("K2", K2, 1.1);
	ROS_INFO("ArDroneControl - Position_control: K2 = %f", K2);

	
	/** creates an object of the declared class. this object will perform the comunication and calculation*/
	GoThroughWindow goWind;

	/** ros::spin() will enter a loop, pumping callbacks.  With this version, all
	* callbacks will be called from within this thread (the main one).  ros::spin()
	* will exit when Ctrl-C is pressed, or the node is shutdown by the master. */
	
	ros::spin();

	return 0;
}

