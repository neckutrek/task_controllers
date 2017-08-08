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

#include <pluginlib/class_list_macros.h>  // to allow the controller to be loaded as a plugin

#include <unistd.h>  // usleep()
#include <iostream>
#include <string>

#include <XmlRpcException.h>
#include <XmlRpcValue.h>

#include <hiqp_ros/hiqp_joint_velocity_controller.h>
#include <hiqp_ros/utilities.h>

#include <hiqp_msgs/StringArray.h>
#include <hiqp_msgs/TaskMeasures.h>
#include <hiqp_msgs/Vector3d.h>

#include <geometry_msgs/PoseStamped.h>  // teleoperation magnet sensors
#include <tf/tfMessage.h>

using hiqp::TaskMeasure;

namespace hiqp_ros {

////////////////////////////////////////////////////////////////////////////////
//  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
////////////////////////////////////////////////////////////////////////////////
//
//              R O S   C O N T R O L L E R   I N T E R F A C E
//
////////////////////////////////////////////////////////////////////////////////

HiQPJointVelocityController::HiQPJointVelocityController()
    : is_active_(true),
      monitoring_active_(false),
      visualizer_(&ros_visualizer_),
      task_manager_(visualizer_),
      task_manager_ptr_(&task_manager_) {}

HiQPJointVelocityController::~HiQPJointVelocityController() noexcept {}

void HiQPJointVelocityController::initialize() {
  ros_visualizer_.init(&(this->getControllerNodeHandle()));
  service_handler_.init(this->getControllerNodeHandlePtr(), task_manager_ptr_,
                        this->getRobotState());

  loadRenderingParameters();

  if (loadAndSetupTaskMonitoring() != 0) return;

  bool tf_primitives = false;
  if (!this->getControllerNodeHandle().getParam("load_primitives_from_tf",
                                                tf_primitives)) {
    ROS_WARN(
        "Couldn't find parameter 'load_primitives_from_tf' on parameter "
        "server, defaulting to no tf primitive tracking.");
  }
  if(tf_primitives) {
      addTfTopicSubscriptions();
  }

  service_handler_.advertiseAll();

  task_manager_.init(getNJoints());

  loadJointLimitsFromParamServer();

  loadGeometricPrimitivesFromParamServer();

  loadTasksFromParamServer();
}

void HiQPJointVelocityController::computeControls(Eigen::VectorXd& u) {
  if (!is_active_) return;

  std::vector<double> outcon(u.size());

  // Time the velocity control computation

  auto t_begin = std::chrono::high_resolution_clock::now();
  task_manager_.getVelocityControls(this->getRobotState(), outcon);
  auto t_end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> opt_time = t_end - t_begin;

  int i = 0;
  for (auto&& oc : outcon) {
    u(i++) = oc;
  }

  renderPrimitives();

  monitorTasks(static_cast<double>(opt_time.count()));

  // if (outcon.size() != 0) {
  //   if (total > 3000.0) {
  //     std::cout << "Velocity Controls computation took " << total / n
  //               << " milliseconds.\n";

  //     fflush(stdout);
  //     n = 0;
  //     total = 0.0;
  //   } else {
  //     n++;
  //     total += static_cast<double>(opt_time.count());
  //   }
  // }

  return;
}

////////////////////////////////////////////////////////////////////////////////
//  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
////////////////////////////////////////////////////////////////////////////////
//
//                      P R I V A T E   M E T H O D S
//
////////////////////////////////////////////////////////////////////////////////

void HiQPJointVelocityController::renderPrimitives() {
  ros::Time now = ros::Time::now();
  ros::Duration d = now - last_rendering_update_;
  if (d.toSec() >= 1.0 / rendering_publish_rate_) {
    last_rendering_update_ = now;
    task_manager_.renderPrimitives();
  }
}

void HiQPJointVelocityController::monitorTasks(double vel_ctl_comp_time) {
  if (monitoring_active_) {
    ros::Time now = ros::Time::now();
    ros::Duration d = now - last_monitoring_update_;
    if (d.toSec() >= 1.0 / monitoring_publish_rate_) {
      last_monitoring_update_ = now;
      std::vector<TaskMeasure> measures;
      task_manager_.getTaskMeasures(measures);

      hiqp_msgs::TaskMeasures msgs;
      msgs.stamp = now;
      for (auto&& measure : measures) {
        hiqp_msgs::TaskMeasure msg;
        msg.task_name = measure.task_name_;
        msg.task_type = measure.task_type_;
        msg.e = std::vector<double>(
            measure.e_.data(),
            measure.e_.data() + measure.e_.rows() * measure.e_.cols());
        msg.de = std::vector<double>(
            measure.de_.data(),
            measure.de_.data() + measure.de_.rows() * measure.de_.cols());
        msg.pm = std::vector<double>(
            measure.pm_.data(),
            measure.pm_.data() + measure.pm_.rows() * measure.pm_.cols());
        msgs.task_measures.push_back(msg);
      }
      msgs.vel_ctl_comp_time = vel_ctl_comp_time;
      if (!msgs.task_measures.empty()) monitoring_pub_.publish(msgs);
    }
  }
}

void HiQPJointVelocityController::loadRenderingParameters() {
  rendering_publish_rate_ = 1000;  // defaults to 1 kHz
  if (!this->getControllerNodeHandle().getParam("visualization_publish_rate",
                                                rendering_publish_rate_)) {
    ROS_WARN(
        "Couldn't find parameter 'visualization_publish_rate' on parameter "
        "server, defaulting to 1 kHz.");
  }
  last_rendering_update_ = ros::Time::now();
}

/// \todo Task monitoring should publish an array of all task infos at each
/// publication time step, rather than indeterministacally publishing single
/// infos on the same topic
int HiQPJointVelocityController::loadAndSetupTaskMonitoring() {
  XmlRpc::XmlRpcValue task_monitoring;
  if (!this->getControllerNodeHandle().getParam("task_monitoring",
                                                task_monitoring)) {
    ROS_ERROR_STREAM("In HiQPJointVelocityController: Call to getParam('"
                     << "task_monitoring"
                     << "') in namespace '"
                     << this->getControllerNodeHandle().getNamespace()
                     << "' failed.");
    return -1;
  }

  int active = static_cast<int>(task_monitoring["active"]);
  monitoring_active_ = (active == 1 ? true : false);
  monitoring_publish_rate_ =
      static_cast<double>(task_monitoring["publish_rate"]);

  monitoring_pub_ =
      this->getControllerNodeHandle().advertise<hiqp_msgs::TaskMeasures>(
          "task_measures", 1);

  return 0;
}

/// \bug Having both, joint limits and avoidance tasks at the highest hierarchy
/// level can cause an infeasible problem (e.g., via starting with
/// yumi_hiqp_preload.yaml tasks)
void HiQPJointVelocityController::loadJointLimitsFromParamServer() {
  XmlRpc::XmlRpcValue hiqp_preload_jnt_limits;
  if (!this->getControllerNodeHandle().getParam("hiqp_preload_jnt_limits",
                                                hiqp_preload_jnt_limits)) {
    ROS_WARN_STREAM("No hiqp_preload_jnt_limits parameter found on "
                    << "the parameter server. No joint limits were loaded!");
  } else {
    bool parsing_success = true;
    for (int i = 0; i < hiqp_preload_jnt_limits.size(); ++i) {
      try {
        std::string link_frame =
            static_cast<std::string>(hiqp_preload_jnt_limits[i]["link_frame"]);

        XmlRpc::XmlRpcValue& limitations =
            hiqp_preload_jnt_limits[i]["limitations"];

        std::vector<std::string> def_params;
        def_params.push_back("TDefJntLimits");
        def_params.push_back(link_frame);
        def_params.push_back(
            std::to_string(static_cast<double>(limitations[1])));
        def_params.push_back(
            std::to_string(static_cast<double>(limitations[2])));

        std::vector<std::string> dyn_params;
        dyn_params.push_back("TDynJntLimits");
        dyn_params.push_back(
            std::to_string(static_cast<double>(limitations[0])));
        dyn_params.push_back(
            std::to_string(static_cast<double>(limitations[3])));

        task_manager_.setTask(link_frame + "_jntlimits", 1, true, true, false,
                              def_params, dyn_params, this->getRobotState());
      } catch (const XmlRpc::XmlRpcException& e) {
        ROS_WARN_STREAM(
            "Error while loading "
            << "hiqp_preload_jnt_limits parameter from the "
            << "parameter server. XmlRcpException thrown with message: "
            << e.getMessage());
        parsing_success = false;
        break;
      }
    }

    if (parsing_success)
      ROS_INFO_STREAM("Loaded and initiated joint limit tasks from .yaml "
                      << "file successfully!");
  }
}

void HiQPJointVelocityController::loadGeometricPrimitivesFromParamServer() {
  XmlRpc::XmlRpcValue hiqp_preload_geometric_primitives;
  if (!this->getControllerNodeHandle().getParam(
          "hiqp_preload_geometric_primitives",
          hiqp_preload_geometric_primitives)) {
    ROS_WARN_STREAM("No hiqp_preload_geometric_primitives parameter "
                    << "found on the parameter server. No geometric primitives "
                    << "were loaded!");
  } else {
    bool parsing_success = true;
    for (int i = 0; i < hiqp_preload_geometric_primitives.size(); ++i) {
      try {
        std::string name = static_cast<std::string>(
            hiqp_preload_geometric_primitives[i]["name"]);
        std::string type = static_cast<std::string>(
            hiqp_preload_geometric_primitives[i]["type"]);
        std::string frame_id = static_cast<std::string>(
            hiqp_preload_geometric_primitives[i]["frame_id"]);
        bool visible =
            static_cast<bool>(hiqp_preload_geometric_primitives[i]["visible"]);

        XmlRpc::XmlRpcValue& color_xml =
            hiqp_preload_geometric_primitives[i]["color"];
        XmlRpc::XmlRpcValue& parameters_xml =
            hiqp_preload_geometric_primitives[i]["parameters"];

        std::vector<double> color;
        color.push_back(static_cast<double>(color_xml[0]));
        color.push_back(static_cast<double>(color_xml[1]));
        color.push_back(static_cast<double>(color_xml[2]));
        color.push_back(static_cast<double>(color_xml[3]));

        std::vector<double> parameters;
        for (int j = 0; j < parameters_xml.size(); ++j) {
          parameters.push_back(static_cast<double>(parameters_xml[j]));
        }

        task_manager_.setPrimitive(name, type, frame_id, visible, color,
                                   parameters);
      } catch (const XmlRpc::XmlRpcException& e) {
        ROS_WARN_STREAM(
            "Error while loading "
            << "hiqp_preload_geometric_primitives parameter from the "
            << "parameter server. XmlRcpException thrown with message: "
            << e.getMessage());
        parsing_success = false;
        break;
      }
    }

    if (parsing_success)
      ROS_INFO_STREAM("Loaded and initiated geometric primitives from "
                      << ".yaml file successfully!");
  }
}

void HiQPJointVelocityController::loadTasksFromParamServer() {
  XmlRpc::XmlRpcValue hiqp_preload_tasks;
  if (!this->getControllerNodeHandle().getParam("hiqp_preload_tasks",
                                                hiqp_preload_tasks)) {
    ROS_WARN_STREAM("No hiqp_preload_tasks parameter found on "
                    << "the parameter server. No tasks were loaded!");
  } else {
    bool parsing_success = true;
    for (int i = 0; i < hiqp_preload_tasks.size(); ++i) {
      try {
        std::string name =
            static_cast<std::string>(hiqp_preload_tasks[i]["name"]);

        XmlRpc::XmlRpcValue& def_params_xml =
            hiqp_preload_tasks[i]["def_params"];
        std::vector<std::string> def_params;
        for (int j = 0; j < def_params_xml.size(); ++j) {
          def_params.push_back(static_cast<std::string>(def_params_xml[j]));
        }

        XmlRpc::XmlRpcValue& dyn_params_xml =
            hiqp_preload_tasks[i]["dyn_params"];
        std::vector<std::string> dyn_params;
        for (int j = 0; j < dyn_params_xml.size(); ++j) {
          dyn_params.push_back(static_cast<std::string>(dyn_params_xml[j]));
        }

        unsigned int priority =
            static_cast<int>(hiqp_preload_tasks[i]["priority"]);
        bool visible = static_cast<bool>(hiqp_preload_tasks[i]["visible"]);
        bool active = static_cast<bool>(hiqp_preload_tasks[i]["active"]);
        bool monitored = static_cast<bool>(hiqp_preload_tasks[i]["monitored"]);

        task_manager_.setTask(name, priority, visible, active, monitored,
                              def_params, dyn_params, this->getRobotState());
      } catch (const XmlRpc::XmlRpcException& e) {
        ROS_WARN_STREAM(
            "Error while loading "
            << "hiqp_preload_tasks parameter from the "
            << "parameter server. XmlRcpException thrown with message: "
            << e.getMessage());
        parsing_success = false;
        break;
      }
    }

    if (parsing_success)
      ROS_INFO("Loaded and initiated tasks from .yaml file successfully!");
  }
}

void HiQPJointVelocityController::addTfTopicSubscriptions()
{
   topic_subscriber_.init( &task_manager_, this->getRobotState() );

   topic_subscriber_.addSubscription<tf::tfMessage>(
     this->getControllerNodeHandle(), "/tf", 100
   );
}

}  // namespace hiqp_ros

// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(hiqp_ros::HiQPJointVelocityController,
                       controller_interface::ControllerBase)
