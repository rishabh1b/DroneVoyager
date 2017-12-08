/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "ardrone_control/PointArray.h"
#include <vector>


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

    for (int i=0; i<msg->markers.size(); ++i)
      {
        /*ROS_INFO_STREAM("detected tag: " << msg->markers[i].id << "position: "<< msg->markers[i].pose.pose.position);*/
        pnt.x=msg->markers[i].pose.pose.position.x;
        pnt.y=msg->markers[i].pose.pose.position.y;
        pnt.z=msg->markers[i].pose.pose.position.z;
        pnts.points.push_back(pnt);
      }

      pub.publish(pnts);
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

