
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

#include "fsm_visual_behavior/Person_Detected.h"
#include "fsm_visual_behavior/bbx_info.h"

namespace fsm_visual_behavior
{

Person_Detected::Person_Detected(const std::string& name)
: BT::ConditionNode(name,{}), n_()
{
  sub_ = n_.subscribe("bbx_custom_topic", 1, &fsm_visual_behavior::Person_Detected::messageCallback, this);
}
void
Person_Detected::messageCallback(const fsm_visual_behavior::bbx_info::ConstPtr& msg){
  dist = msg->dist;
}

BT::NodeStatus
Person_Detected::tick()
{
  if (!std::isnan(dist))
  {
    ROS_INFO("%f hola",dist);
    return BT::NodeStatus::SUCCESS;
  }
  else  
  {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace fsm_visual_behavior
