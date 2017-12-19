# DroneVoyager
This software was developed for a Project in course _CMSC828T - Perception, Planning and Control for Aerial Robots_ at the University of Maryland. The task which was attempted to complete in this project was to drive a AR drone through a window. The window had AR tag markers to aid the detection. The existing software will try to locate the AR markers in a pattern of four at the corners of the window and try to fly towards the centre of the window

_DISCLAIMER_:- The software is not perfect and it does not function reliably always on the expected lines. Having said that drone does not perform any aggressive manuevers and normal safety precautions should be followed while working with the drone.

# Dependencies
1. ROS Indigo. Instructions are [here](http://wiki.ros.org/indigo/Installation/Ubuntu)
2. AR alvar package for AR tag detection - 
```sudo apt-get install ros-indigo-ar-track-alvar```
# Installation
1. Create a workspace - 
```
mkdir -p ~/ARDroneWindow/src
cd  ~/ARDroneWindow/src
catkin_init_workspace
```
2. Download Dependencies - 
```
git clone https://github.com/AutonomyLab/ardrone_autonomy.git	# The AR.Drone ROS driver
git clone https://github.com/occomco/tum_simulator.git
cd ..
rosdep install --from-paths src --ignore-src --rosdistro indigo -y
```
3. Download Control and Perception Software from this repo -
```
cd src
git clone https://github.com/rishabh1b/DroneVoyager
cd..
```
4. Build the software - 
```catkin_make```
5. Source the setup file - 
```source devel/setup.bash```

# Running a demo on Real Drone -
There are some prerequisites that should be taken care of before running on real drone
1. Make sure you have AR tag markers located in vicinity of the drone. The tags can be downloaded from the page[here](http://wiki.ros.org/ar_track_alvar). 
2. Enter the tag size you printed is entered in ```ar_drone_percept_test.launch``` file.
3. Finally, you must calibrate the front camera of the drone using [ros_camera_calibration](http://wiki.ros.org/camera_calibration) and put the calibration file in ```~/.ros/camera_info/ardrone_front.yaml```or use the one in this repo

Once the above steps are completed. Do the following -
1. Connect to the AR drone wifi
2. Terminal 1 - 
```roscore```
3. Terminal 2[Run the Driver] - 
```rosrun ardrone_autonomy ardrone_driver```
4. Terminal 3[Takeoff] - 
```rostopic pub -1 ardrone/takeoff std_msgs/Empty ```
5. Terminal 4[Running current software launch file] - 
```roslaunch ardrone_control ardrone_percept_test.launch```
After running the launch file, you can give the control to the drone by pressing the ```G``` key when focused in this terminal.
To withdraw the control to the drone, you can press the ```M``` key when focused in this terminal
6. Terminal 5[Keep Landing handy]
```rostopic pub -1 ardrone/land std_msgs/Empty```
7. Terminal 6[rviz]
```rosrun rviz rviz```

# Demonstration
Here is the YouTube [link](https://youtu.be/PYlbKkE2rPg) of our attempt with this software.

## Simulation using gazebo/tum_simulator
A gazebo simulator environment was built to test control codes with the tum_simulator package. This package depends on and mimics the ardrone_autonomy package. Thus when running the tum_simulator, the ardrone_autonomy package is running in the background and publishing to the same topics it would as if ardrone_autonomy were controlling an actual ardrone. This allows the use of the ar_track_alvar package which uses the /ardrone/front/image_raw topic to detect the tags seen by the drone camera (in this case, what the drone in the simulator "sees".).

The files in the simulator folder can be used to simulate the flight of the AR Drone reading AR tags and flying to the window. This simulation was developed using ROS indigo on Ubuntu 14.04 and gazebo 2. It may be possible to run gazebo 7 with ROS indigo but this was not explored. NOTE: The ar tags included in these files are markers 0,2,3,5 because they matched what we used to test the real drone. Also note, the size of the tags in this simulator environment cannot be changed as gazebo 2 does not have this functionality. Gazebo 7 might be able to scale them down to a specified sized but this was not explored.

### Copy Simulator Files
copy folders marker0 marker2 marker3 marker5 to ```~/.gazebo/models```

copy project.launch to ```ARDroneWindow/src/tum_simulator/cvg_sim_gazebo/launch```

copy tags.world to the ```ARDroneWindow/src/tum_simulator/cvg_sim_gazebo/worlds```


### To Run Simulator
1) source ros installation
2) source catkin ws with simulator
```
source ArDroneWindow/devel/setup.bash
```
3) launch simulator with drone and ar tags
```
roslaunch cvg_sim_gazebo project.launch
```
4) launch DroneVoyager control node
```
roslaunch ardrone_control ardrone_percept_test.launch
```




