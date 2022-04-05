
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

#ifndef FSM_VISUAL_BEHAVIOR_TURN_H
#define FSM_VISUAL_BEHAVIOR_TURN_H

#include <string>
#include "ros/ros.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "geometry_msgs/Twist.h"
#include "geometry_tf/transforms.h"

namespace fsm_visual_behavior
{

class Turn : public BT::ActionNodeBase
{
  public:
    ros::NodeHandle n_;
    explicit Turn(const std::string& name);

    void halt();

    BT::NodeStatus tick();

  private:
    static constexpr float TURNING_VEL = 0.5;
    static constexpr double TURNING_TIME = 2.5;
    ros::Publisher pub_vel_;

};

}  // namespace fsm_visual_behavior

#endif  // FSM_VISUAL_BEHAVIOR_TURN_H
