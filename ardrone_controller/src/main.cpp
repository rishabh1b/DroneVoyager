#include "ardrone_controller/basicControl.h"

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "openloop");
	ros::NodeHandle n;
	OpenLoopControl ol = OpenLoopControl(n);
    ol.setWaypoint(1);
    ol.execute();
}