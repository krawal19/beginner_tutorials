/**
 * BSD 3-Clause License


 * Copyright (c) 2018, KapilRawal
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *  @copyright (c) BSD
 *
 *  @file   talker.cpp
 *
 *  @author   Kapil Rawal
 *
 *  @copyright   BSD License
 *
 *  @brief   ROS listener node
 *
 *  @section   DESCRIPTION
 *
 *  This program publishes the message on the topic chatter and produces world and talk frames
 *
 */

#include <tf/transform_broadcaster.h>
#include <sstream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/serviceFile.h"

/// Definer value of pi for easy access
#define PI 3.14;

struct Sentmsg {
  std::string msgSent;
}message;

/**
 *  @brief changes base string of talker
 *
 *  @param req requesting new string from service client
 *
 *  @param res response of the service from client
 *
 *  @return true service function executed
 */
bool change(beginner_tutorials::serviceFile::Request& req,
            beginner_tutorials::serviceFile::Response& res) {
  res.changedString = req.newString;
  message.msgSent = message.msgSent + req.newString;
  return true;
}

/**
 *  @brief   This tutorial demonstrates simple Receving of messages over the ROS system.
 *
 *  @param   argc  Count of arguments given on terminal
 *
 *  @param   argv  Pointer to the array of pointers pointing to the argument given in the terminal
 *
 *  @return  returns nothing
 */
int main(int argc, char **argv) {
  /// Initializes the talker node
  ros::init(argc, argv, "talker");

  ///  Creating a TransformBroadcaster object,
  ///  to send transformation over the wire
  static tf::TransformBroadcaster br;

  ///  Creating Transform object
  tf::Transform transform;

  /// quaternion variable
  tf::Quaternion q;

  /// Setting the radius
  double r = 1.0;

  /// Setting the angular velocity
  double w = 2 * PI

  /// Declaring result as string
  std::string result;
 
  message.msgSent = "Did you pass 808x? ";

  /// Checking if we have two arguments
  if (argc == 2) {
    /// recording the argument passed
    ROS_DEBUG_STREAM("Argument is " << argv[1]);
    result = argv[1];
    if (result.empty()) {
      ROS_ERROR_STREAM("No was given, using default string");
      result = "Didn't took the course";
    }
  }
/// else {
/// shuts down the node when no argument given
///    ROS_FATAL_STREAM("Not applicable for this student");
///    ros::shutdown();
///    return 1;
///  }

  /// Creating a handle to process
  ros::NodeHandle n;

  /// defines the topic and type of message on
  /// which the publisher is going to publish data
  ros::Publisher chatter_pub = n.advertise < std_msgs::String
      > ("chatter", 1000);
  /// defining service server
  ros::ServiceServer server = n.advertiseService("serviceFile", change);
  /// Rate of loop running.
  ros::Rate loop_rate(10);
  /// Number of messages that have been sent.
  int count = 0;
  while (ros::ok()) {
    /// Find the time
    double t = ros::Time::now().toSec();

    /// Get the coordinates of the circle
    double x = r * (sin(w * t) + cos(w * t));
    double y = r * (sin(w * t) - cos(w * t));
    double z = 0.0;
    double theta = w * t;

    ///  Copying information form the 2D turtle pose into the 3D transform
    transform.setOrigin(tf::Vector3(x, y, z));

    /// Set the orientation
    q.setRPY(0, 0, theta);

    ///  Setting rotation
    transform.setRotation(q);

    /** @brief Sending the transform
     *
     *  @param Passing transform itself
     *
     *  @param Giving timestamp to the transform being published(giving current time)
     *
     *  @param Name of the parent frame of the link we created(world)
     *
     *  @param Passing name of the child frame of the link we created(talk)
     */
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));

    /// Creating a message object and publish it.
    std_msgs::String msg;
    std::stringstream ss;
    ss << message.msgSent + result << count;
    msg.data = ss.str();

    /// print data on terminal
    ROS_INFO("%s", msg.data.c_str());

    /// message output as broadcast
    chatter_pub.publish(msg);

    /// for subscriber node
    ros::spinOnce();

    /// when not publishing in sleep mode to match the publish rate
    loop_rate.sleep();
    ++count;
    // ROS_DEBUG_STREAM("Argument is " << count);
  }

  return 0;
}

