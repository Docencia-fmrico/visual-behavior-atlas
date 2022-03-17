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

#ifndef FSM_VISUAL_PERSON_PERSONFOUND_H
#define FSM_VISUAL_PERSON_PERSONFOUND_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "darknet_ros_msgs/BoundingBoxes.h"

#include <string>

namespace behavior_trees
{

class PersonFound : public BT::ActionNodeBase
{
  public:
    explicit PersonFound(const std::string& name);

    void halt();

    void boxesCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);

    BT::NodeStatus tick();

  private:
    int counter_;
    int coordX_;
    int coordY_;
    bool found_ = false;
    ros::NodeHandle n_;
    ros::Subscriber sub_boxes_;
};

}  // namespace behavior_trees

#endif  // FSM_VISUAL_PERSON_PERSONFOUND_H