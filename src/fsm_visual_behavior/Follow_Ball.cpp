
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

#include "fsm_visual_behavior/Follow_Ball.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/Twist.h"

#include "ros/ros.h"

namespace fsm_visual_behavior
{

Follow_Ball::Follow_Ball(const std::string& name)
: BT::ActionNodeBase(name, {}), n_(), buffer(), listener(buffer), velocity_pid(0.0, 5.0, 0.0, 0.20), turn_pid(0.0, 3.0, 0.0, 0.5)
{
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
}

void
Follow_Ball::halt()
{
  ROS_INFO("finished going forward");
}

BT::NodeStatus
Follow_Ball::tick()
{
  geometry_msgs::Twist vel_msgs;

  geometry_msgs::TransformStamped bf2ball_msg;
  tf2::Stamped<tf2::Transform> bf2ball;
  std::string error;

  bf2ball_msg = buffer.lookupTransform("base_footprint", "object/0", ros::Time(0));

  tf2::fromMsg(bf2ball_msg, bf2ball);

  double dist = bf2ball.getOrigin().length();
  double angle = atan2(bf2ball.getOrigin().y(), bf2ball.getOrigin().x());

  ROS_INFO("base_footprint -> ball [%lf, %lf] dist = %lf  angle = %lf       %lf ago",
    bf2ball.getOrigin().x(),
    bf2ball.getOrigin().y(),
    dist,
    angle,
    (ros::Time::now() - bf2ball.stamp_).toSec());

      
  double speed_clamped = std::clamp(dist -1, -5.0, 5.0);
  double angle_clamped = std::clamp(angle, -1.0, 1.0)*10;

  float speed_pid = velocity_pid.get_output(speed_clamped)*1.0f;
  float angle_pid = turn_pid.get_output(angle_clamped)*1.0f;

  vel_msgs.angular.z = angle_pid;
  vel_msgs.linear.x = speed_pid;

  ROS_INFO("angle_pid = %f angle clamped = %lf",angle_pid, angle_clamped);
  ROS_INFO("spid_pid = %f",speed_pid);
  pub_vel_.publish(vel_msgs);
  return BT::NodeStatus::RUNNING;
}

}  // namespace fsm_visual_behavior
