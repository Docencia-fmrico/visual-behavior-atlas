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

#include "fsm_visual_person/PersonFound.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"

namespace behavior_trees
{

PersonFound::PersonFound(const std::string& name)
: BT::ActionNodeBase(name, {}), counter_(0)
{
  sub_boxes_ = n_.subscribe("/darknet_ros/bounding_boxes", 0, &PersonFound::boxesCallback, this);
}

void
PersonFound::halt()
{
  ROS_INFO("PersonFound halt");
}

void 
PersonFound::boxesCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  std::cout <<"Class: " << msg->bounding_boxes[0].Class << std::endl;
  
  if(msg->bounding_boxes[0].Class == "person")
  {
    found_ = true;
    coordX_ = ((msg->bounding_boxes[0].xmin) + (msg->bounding_boxes[0].xmax))/2;
    coordY_ = ((msg->bounding_boxes[0].ymin) + (msg->bounding_boxes[0].ymax))/2;
  }
}

BT::NodeStatus
PersonFound::tick()
{
  std::cout << "x: " << coordX_ << " y: " << coordY_ << " enc: " << found_ << std::endl;
  ROS_INFO("PersonFound tick");
  if(found_)
  {
    found_ = false;
    return BT::NodeStatus::SUCCESS;
  }
  else 
  {
    return BT::NodeStatus::FAILURE;
  }
  
}

}  // namespace behavior_trees