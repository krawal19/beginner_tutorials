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
 *  This program publishes the message 'You are the best' on the topic chatter
 *
 */

#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 *  @brief   This tutorial demonstrates simple sending of messages over the ROS system.
 *
 *  @param   argc  Count of arguments given on terminal
 *
 *  @param   argv  Pointer to the array of pointers pointing to the argument given in the terminal
 *
 *  @return  returns nothing
 */
int main(int argc, char **argv) {
  /// Initialize function for the node, for name remapping and
  /// to perform any ROS arguments
  ros::init(argc, argv, "talker");
  ///  Create a instance of NodeHandle
  ros::NodeHandle n;

  ///  Create a Publisher node and define topic named chatter
  ///  Publish message to topic with queue size of 1000
  ros::Publisher chatter_pub = n.advertise < std_msgs::String
      > ("chatter", 1000);

  ///  run loop at 10Hz rate
  ros::Rate loop_rate(10);

  ///  count of sent message
  int count = 0;

  ///  loop until node is shutdown
  while (ros::ok()) {
    /// create String message
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Your are the best" << count;

    ///  store custom message string
    msg.data = ss.str();

    ///  display message
    ROS_INFO("%s", msg.data.c_str());

    ///  using publisher object to publish messages
    chatter_pub.publish(msg);

    ///  asks ROS to execute all of the pending
    ///  callbacks from all node's subscriptions
    ros::spinOnce();

    ///  Sleep the publishing till next iteration
    loop_rate.sleep();
    ++count;
  }
  return 0;
}

