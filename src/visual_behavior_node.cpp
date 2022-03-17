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

#include "visual_behavior/FollowPerson.h"
#include "visual_behavior/PersonNotFound.h"
#include "visual_behavior/SearchPerson.h"

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/package.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavior_tree");
  ros::NodeHandle n;

  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<behavior_tree_person::FollowPerson>("FollowPerson");
  factory.registerNodeType<behavior_tree_person::SearchPerson>("SearchPerson");
  factory.registerNodeType<behavior_tree_person::PersonNotFound>("PersonNotFound");

  auto blackboard = BT::Blackboard::create();

  blackboard->set("object", "cup");

  std::string pkgpath = ros::package::getPath("visual_behavior");
  std::string xml_file = pkgpath + "/visual_behavior_xml/tree.xml";

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  ros::Rate loop_rate(5);

  int count = 0;

  bool finish = false;
  while (ros::ok() && !finish)
  {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}