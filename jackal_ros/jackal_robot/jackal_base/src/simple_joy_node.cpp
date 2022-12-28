/**
Software License Agreement (BSD)

\file      joy_pwm_node.cpp
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "ros/ros.h"
#include "jackal_msgs/Drive.h"
#include "sensor_msgs/Joy.h"

#include "boost/algorithm/clamp.hpp"

namespace jackal_teleop
{

class SimpleJoy
{
public:
  explicit SimpleJoy(ros::NodeHandle* nh);

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle* nh_;
  ros::Subscriber joy_sub_;
  ros::Publisher drive_pub_;

  int deadman_button_;
  int axis_linear_;
  int axis_angular_;
  float scale_linear_;
  float scale_angular_;

  bool sent_deadman_msg_;
};

SimpleJoy::SimpleJoy(ros::NodeHandle* nh) : nh_(nh)
{
  joy_sub_ = nh_->subscribe<sensor_msgs::Joy>("joy", 1, &SimpleJoy::joyCallback, this);
  drive_pub_ = nh_->advertise<jackal_msgs::Drive>("cmd_drive", 1, true);

  ros::param::param("~deadman_button", deadman_button_, 0);
  ros::param::param("~axis_linear", axis_linear_, 1);
  ros::param::param("~axis_angular", axis_angular_, 0);
  ros::param::param("~scale_linear", scale_linear_, 1.0f);
  ros::param::param("~scale_angular", scale_angular_, 1.0f);

  sent_deadman_msg_ = false;
}

void SimpleJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  jackal_msgs::Drive drive_msg;

  if (joy_msg->buttons[deadman_button_])
  {
    drive_msg.mode = jackal_msgs::Drive::MODE_PWM;
    float linear = joy_msg->axes[axis_linear_] * scale_linear_;
    float angular = joy_msg->axes[axis_angular_] * scale_angular_;
    drive_msg.drivers[jackal_msgs::Drive::LEFT] = boost::algorithm::clamp(linear - angular, -1.0, 1.0);
    drive_msg.drivers[jackal_msgs::Drive::RIGHT] = boost::algorithm::clamp(linear + angular, -1.0, 1.0);
    drive_pub_.publish(drive_msg);
    sent_deadman_msg_ = false;
  }
  else
  {
    // When deadman button is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (!sent_deadman_msg_)
    {
      drive_msg.mode = jackal_msgs::Drive::MODE_NONE;
      drive_pub_.publish(drive_msg);
      sent_deadman_msg_ = true;
    }
  }
}

}  // namespace jackal_teleop

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "jackal_teleop_joy_pwm");

  ros::NodeHandle nh;
  jackal_teleop::SimpleJoy simple_joy(&nh);

  ros::spin();
}
