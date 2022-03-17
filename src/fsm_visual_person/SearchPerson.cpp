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

#include "fsm_visual_person/SearchPerson.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"

#include "geometry_msgs/Twist.h"

namespace behavior_trees
{

SearchPerson::SearchPerson(const std::string& name)
: BT::ActionNodeBase(name, {}), counter_(0)
{
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
  cmd_.angular.z = 0.4;
}

void
SearchPerson::halt()
{
  ROS_INFO("SearchPerson halt");
}

BT::NodeStatus
SearchPerson::tick()
{
  ROS_INFO("SearchPerson tick");
  
  pub_vel_.publish(cmd_);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace behavior_trees