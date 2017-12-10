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
#include "ardrone_control/PointArray.h"
#include <vector>
#include <valarray>
#include <iostream>


class ReadAndCalc
{
public:
  ReadAndCalc()
  {
    pub = n.advertise<ardrone_control::PointArray>("window_cent", 100);
    sub = n.subscribe("ar_pose_marker", 100, &ReadAndCalc::chatterCallback, this);
  }

  void chatterCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
  {

    geometry_msgs::Point pnt;
    ardrone_control::PointArray pnts;
    std::vector<float> x,y,z,d,e,f;
    /*std::vector<float> y;*/
    /*std::vector<float> d;*/
    /*std::vector<float> e;*/
    float dMax=0.0;
    float eMax=0.0;
    float fMax=0.0;

    for (int i=0; i<msg->markers.size(); ++i)
      {
        /*ROS_INFO_STREAM("detected tag: " << msg->markers[i].id << "position: "<< msg->markers[i].pose.pose.position);*/

        /* WHAT IF NOT ALL 4 TAGS ARE DETECTED? 3 IS ENOUGH TO FIND WINDOW CENTER. LOOK AT USING BUNDLES*/
        /* WHAT IF THE 5 TAG IS ACCIDENTALLY DETECTED?*/
        /* HOW TO DEFINE THE WINDO TAGS SEPARATELY FROM 5TH TAG?*/
        x.push_back(msg->markers[i].pose.pose.position.x);
        y.push_back(msg->markers[i].pose.pose.position.y);
        z.push_back(msg->markers[i].pose.pose.position.z);
        /*pnts.points.push_back(pnt);*/
      }
      
    for (int j=1; j<x.size(); ++j) 
      {  
        d.push_back(x[0]-x[j]);
        e.push_back(y[0]-y[j]);
        f.push_back(z[0]-z[j]);
        /*ROS_INFO("d : %d",d);
        /*std::valarray<float> dist (val,3);*/
        /*std::valarray<float> cent = abs(dist);*/
      }
    /*ROS_INFO("size of d : %d",d.size());
      /*std::cout << d.size();*/
    for (int k = 0; k < d.size(); k++)
      {
        if (abs(d[k]) > dMax)
          pnt.x = abs(d[k]);
        if (abs(e[k]) > eMax)
          pnt.y = (e[k]);
        if (abs(f[k]) > fMax)
          pnt.z = f[k];
      }
    if (x.size() > 0)
    { 
      pnt.x+=x[0];
      pnt.y+=y[0];
      pnt.z+=z[0];
      pnts.points.push_back(pnt);  
      pub.publish(pnts);
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

