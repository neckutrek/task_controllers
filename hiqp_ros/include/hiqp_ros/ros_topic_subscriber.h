// The HiQP Control Framework, an optimal control framework targeted at robotics
// Copyright (C) 2016 Marcus A Johansson
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef HIQP_ROS_TOPIC_SUBSCRIBER_H
#define HIQP_ROS_TOPIC_SUBSCRIBER_H

#include <string>
#include <vector>

#include <hiqp/geometric_primitives/geometric_cylinder.h>
#include <hiqp/task_manager.h>
#include <hiqp/robot_state.h>

#include "ros/ros.h"

using hiqp::geometric_primitives::GeometricPrimitiveMap;
using hiqp::geometric_primitives::GeometricPoint;
using hiqp::geometric_primitives::GeometricLine;
using hiqp::geometric_primitives::GeometricPlane;
using hiqp::geometric_primitives::GeometricBox;
using hiqp::geometric_primitives::GeometricCylinder;
using hiqp::geometric_primitives::GeometricSphere;
using hiqp::geometric_primitives::GeometricFrame;

namespace hiqp_ros {

class ROSTopicSubscriber {
 public:
  ROSTopicSubscriber() {}
  ~ROSTopicSubscriber() {}

  int init(hiqp::TaskManager* task_manager, hiqp::RobotStatePtr robot_state_ptr) { 
      task_manager_ = task_manager; 
      robot_state_ptr_ = robot_state_ptr;
  }

  template <typename ROSMessageType>
  int addSubscription(ros::NodeHandle& controller_nh,
                      const std::string& topic_name, unsigned int buffer_size) {
    sub = controller_nh.subscribe(
        topic_name, buffer_size,
        &ROSTopicSubscriber::topicCallback<ROSMessageType>, this);
    ROS_INFO_STREAM("Subsribed to topic '" << topic_name << "'");
  }

  /*! \brief Implement this function for your own message! */
  template <typename ROSMessageType>
  void topicCallback(const ROSMessageType& msg);

 private:
  ROSTopicSubscriber(const ROSTopicSubscriber& other) = delete;
  ROSTopicSubscriber(ROSTopicSubscriber&& other) = delete;
  ROSTopicSubscriber& operator=(const ROSTopicSubscriber& other) = delete;
  ROSTopicSubscriber& operator=(ROSTopicSubscriber&& other) noexcept = delete;

  ros::Subscriber sub;

  hiqp::TaskManager* task_manager_;
  hiqp::RobotStatePtr robot_state_ptr_;
};

}  // namespace hiqp

#endif  // Include guard
