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

#ifndef HIQP_JOINT_VELOCITY_CONTROLLER_H
#define HIQP_JOINT_VELOCITY_CONTROLLER_H

#include <string>
#include <vector>

#include <hardware_interface/joint_command_interface.h>
#include <ros/ros.h>

#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <hiqp/hiqp_time_point.h>
#include <hiqp/task_manager.h>

#include <hiqp_ros/base_controller.h>
#include <hiqp_ros/hiqp_service_handler.h>
#include <hiqp_ros/ros_topic_subscriber.h>
#include <hiqp_ros/ros_visualizer.h>

#include <fstream>

namespace hiqp_ros {
typedef hardware_interface::VelocityJointInterface JointVelocityInterface;

/*! \brief A joint velocity controller that provides full access to the HiQP
 * control framework
 *  \author Marcus A Johansson */
class HiQPJointVelocityController
    : public BaseController<JointVelocityInterface> {
 public:
  HiQPJointVelocityController();
  ~HiQPJointVelocityController() noexcept;

  void initialize();
  void computeControls(Eigen::VectorXd& u);

 private:
  HiQPJointVelocityController(const HiQPJointVelocityController& other) =
      delete;
  HiQPJointVelocityController(HiQPJointVelocityController&& other) = delete;
  HiQPJointVelocityController& operator=(
      const HiQPJointVelocityController& other) = delete;
  HiQPJointVelocityController& operator=(
      HiQPJointVelocityController&& other) noexcept = delete;

  void monitorTasks(double vel_ctl_comp_time);
  void renderPrimitives();

  void loadRenderingParameters();
  int loadAndSetupTaskMonitoring();
  void addTfTopicSubscriptions();
  void loadJointLimitsFromParamServer();
  void loadGeometricPrimitivesFromParamServer();
  void loadTasksFromParamServer();

  bool is_active_;

  bool monitoring_active_;
  double monitoring_publish_rate_;
  ros::Time last_monitoring_update_;

  double rendering_publish_rate_;
  ros::Time last_rendering_update_;

  ros::Publisher monitoring_pub_;

  ROSTopicSubscriber topic_subscriber_;

  HiQPServiceHandler service_handler_;  // takes care of all ros service calls

  ROSVisualizer ros_visualizer_;
  std::shared_ptr<Visualizer> visualizer_;

  hiqp::TaskManager task_manager_;
  std::shared_ptr<hiqp::TaskManager> task_manager_ptr_;
};

}  // namespace hiqp

#endif  // include guard
