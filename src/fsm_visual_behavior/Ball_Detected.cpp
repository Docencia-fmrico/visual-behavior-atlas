
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

#include "fsm_visual_behavior/Ball_Detected.h"

namespace fsm_visual_behavior
{

Ball_Detected::Ball_Detected(const std::string& name)
: BT::ConditionNode(name,{}), n_(), buffer(), listener(buffer)
{}

BT::NodeStatus
Ball_Detected::tick()
{
  geometry_msgs::TransformStamped bf2ball_msg;
  tf2::Stamped<tf2::Transform> bf2ball;
  if(buffer.canTransform("base_footprint", "object/0", ros::Time(0), ros::Duration(4.0), &error))
  {
    bf2ball_msg = buffer.lookupTransform("base_footprint", "object/0", ros::Time(0));
    tf2::fromMsg(bf2ball_msg, bf2ball);
    if( (ros::Time::now() - bf2ball.stamp_).toSec() < 1.0f){
      return BT::NodeStatus::SUCCESS;
    }
    else{
      return BT::NodeStatus::FAILURE;
    }
  }
  else  
  {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace fsm_visual_behavior
