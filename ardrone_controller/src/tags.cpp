/*
 * Michael Avera
 * University of Maryland, College Park
 * CMSC828T Fall 2017
 * Project (Fly AR Drone 2.0 through window with AR tags at each corner)
 *
 * TAGS NODE (this file)
 * Identify AR tags of window and calculate location of window center relative to camera frame
 * reads detected AR tag locations published BY ar_track_alvar ROS package to /ar_pose_marker topic
 * publishes 3D coordinates of window center to /window_cent topic 
 */

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include <vector>
#include <iostream>
#include <cmath>


class ReadAndCalc
{
public:
  ReadAndCalc()
  {
    pub = n.advertise<geometry_msgs::Point>("window_cent", 100);
    sub = n.subscribe("ar_pose_marker", 100, &ReadAndCalc::chatterCallback, this);
  }

  void chatterCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
  {

    geometry_msgs::Point pnt;
    std::vector<float> x,y,z,d,e,f;
    float dMax=0.0;
    float eMax=0.0;
    float fMax=0.0;

    for (int i=0; i<msg->markers.size(); ++i)
      {

        /* WHAT IF NOT ALL 4 TAGS ARE DETECTED? 3 IS ENOUGH TO FIND WINDOW CENTER. LOOK AT USING BUNDLES*/
        /* WHAT IF THE 5 TAG IS ACCIDENTALLY DETECTED?*/
        /* HOW TO DEFINE THE WINDO TAGS SEPARATELY FROM 5TH TAG?*/
        x.push_back(msg->markers[i].pose.pose.position.x);
        y.push_back(msg->markers[i].pose.pose.position.y);
        z.push_back(msg->markers[i].pose.pose.position.z);
        if (i>0)
        {
          d.push_back(x[0]-x[i]);
          e.push_back(y[0]-y[i]);
          f.push_back(z[0]-z[i]);
        if (std::abs(d[i-1]) > dMax) {
          // pnt.x = std::abs(d[i])/2+x[0];
          pnt.x = std::abs(d[i-1])/2;
          dMax = std::abs(d[i-1]);
        }
        if (std::abs(e[i-1]) > eMax) {
          // pnt.y = std::abs(e[i])/2+y[0];
          pnt.y = std::abs(e[i-1])/2;
          eMax = std::abs(e[i-1]);
        }
        if (std::abs(f[i-1]) > fMax) {
          // pnt.z = std::abs(f[i])/2+z[0];
          pnt.z = std::abs(f[i-1])/2;
          fMax = std::abs(f[i-1]);
        }
      }
      //if (x.size()>0)
      pub.publish(pnt);   
  }
}
// %EndTag(CALLBACK)%

private:
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub;
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "tags");

  ReadAndCalc Window;

  ros::spin();

  return 0;
}

