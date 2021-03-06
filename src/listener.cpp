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
 *  @file listener.cpp
 *
 *  @author Kapil Rawal
 *
 *  @copyright BSD License
 *
 *  @brief ROS listener node
 *
 *  @section DESCRIPTION
 *
 *  This program subscribe to the topic chatter to display the message sent by publisher
 *
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * @brief A callback function which is called when data arrives on topic
 *
 * @param msg takes input string published by talker
 *
 * @return void
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: [%s]", msg->data.c_str());
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
  ///  Initializer function for the node
  ros::init(argc, argv, "listener");

  ///  Create a instance of NodeHandle
  ros::NodeHandle n;

  ///  Subscribing to the chatter topic
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ///  ask ROS to wait for and execute callbacks until the node shuts down
  ros::spin();
  return 0;
}
