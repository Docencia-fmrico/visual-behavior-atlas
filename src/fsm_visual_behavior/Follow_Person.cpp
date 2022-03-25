
// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>

#include "fsm_visual_behavior/Follow_Person.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/Twist.h"

#include "ros/ros.h"

namespace fsm_visual_behavior
{

Follow_Person::Follow_Person(const std::string& name)
: BT::ActionNodeBase(name, {}), velocity_pid(0.0, 5.0, 0.0, 0.5), turn_pid(0.0, 3.2, 0.0, 0.15)
{
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
  sub_ = n_.subscribe("bbx_custom_topic", 1, &fsm_visual_behavior::Follow_Person::messageCallback, this);
}

void
Follow_Person::messageCallback(const fsm_visual_behavior::bbx_info::ConstPtr& msg){
  dist = msg->dist;
  px = msg->px;
  
}

void
Follow_Person::halt()
{
  ROS_INFO("finished going forward");
}

BT::NodeStatus
Follow_Person::tick()
{
  geometry_msgs::Twist vel_msgs;
  double speed_filered = std::clamp(dist -1, -5.0, 5.0);

  double angle_filered = (px-320)*(-100.0);

  vel_msgs.linear.x = velocity_pid.get_output(speed_filered);
  vel_msgs.angular.z = turn_pid.get_output(angle_filered);
      
  pub_vel_.publish(vel_msgs);
  
  return BT::NodeStatus::RUNNING;
}

}  // namespace fsm_visual_behavior
