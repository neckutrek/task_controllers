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

#include <pluginlib/class_list_macros.h> // to allow the controller to be loaded as a plugin

#include <iostream>
#include <string>
#include <unistd.h> // usleep()

#include <XmlRpcValue.h>  
#include <XmlRpcException.h> 

#include <hiqp_ros/utilities.h>
#include <hiqp_ros/hiqp_joint_velocity_controller.h>

#include <hiqp_msgs/TaskMeasures.h>
#include <hiqp_msgs/Vector3d.h>
#include <hiqp_msgs/StringArray.h>

#include <geometry_msgs/PoseStamped.h> // teleoperation magnet sensors
#include <hiqp_msgs/PoseWithName.h> // teleoperation magnet sensors

using hiqp::TaskMeasure;

namespace hiqp_ros
{

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
: visualizer_(&ros_visualizer_),
  is_active_(true), monitoring_active_(false), task_manager_(visualizer_) {}

HiQPJointVelocityController::~HiQPJointVelocityController() noexcept {}

void HiQPJointVelocityController::initialize() {
  ros_visualizer_.init( &(this->getControllerNodeHandle()) );

  loadRenderingParameters();

  if (loadAndSetupTaskMonitoring() != 0) return;

  addAllTopicSubscriptions();

  advertiseAllServices();

  task_manager_.init(getNJoints());

  loadJointLimitsFromParamServer();

  loadGeometricPrimitivesFromParamServer();

  loadTasksFromParamServer();
}





void HiQPJointVelocityController::computeControls(Eigen::VectorXd& u) {
  if (!is_active_) return;

  std::vector<double> outcon(u.size());
  service_mutex_.lock();
  task_manager_.getVelocityControls(this->getRobotState(), outcon);
  service_mutex_.unlock();
  int i=0;
  for (auto&& oc : outcon) {
    u(i++) = oc;
  }

  service_mutex_.lock();
  renderPrimitives();
  service_mutex_.unlock();

  service_mutex_.lock();
  monitorTasks();
  service_mutex_.unlock();

  return;
}









////////////////////////////////////////////////////////////////////////////////
//  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
////////////////////////////////////////////////////////////////////////////////
//
//                R O S   S E R V I C E   C A L L B A C K S
//
////////////////////////////////////////////////////////////////////////////////

bool HiQPJointVelocityController::setTask(hiqp_msgs::SetTask::Request& req, 
                                          hiqp_msgs::SetTask::Response& res) {
  service_mutex_.lock();
  int retval = task_manager_.setTask(
    req.name, req.priority, req.visible, req.active,
    req.def_params, req.dyn_params, this->getRobotState());
  res.success = (retval < 0 ? false : true);
  service_mutex_.unlock();
  return true;
}

bool HiQPJointVelocityController::removeTask(hiqp_msgs::RemoveTask::Request& req, 
                                             hiqp_msgs::RemoveTask::Response& res) {
  service_mutex_.lock();
  res.success = false;
  if (task_manager_.removeTask(req.task_name) == 0)
    res.success = true;

  if (res.success) {
    hiqp::printHiqpInfo("Removed task '" + req.task_name + "'.");
  } else {
    hiqp::printHiqpInfo("Couldn't remove task '" + req.task_name + "'!");  
  }
  service_mutex_.unlock();
  return true;
}

bool HiQPJointVelocityController::removeAllTasks(hiqp_msgs::RemoveAllTasks::Request& req, 
                                                 hiqp_msgs::RemoveAllTasks::Response& res) {
  service_mutex_.lock();
  task_manager_.removeAllTasks();
  hiqp::printHiqpInfo("Removed all tasks successfully!");
  res.success = true;
  service_mutex_.unlock();
  return true;
}

bool HiQPJointVelocityController::listAllTasks(hiqp_msgs::ListAllTasks::Request& req, 
                                               hiqp_msgs::ListAllTasks::Response& res) {
  service_mutex_.lock();
  task_manager_.listAllTasks();
  res.success = true;
  service_mutex_.unlock();
  return true;
}

bool HiQPJointVelocityController::addGeometricPrimitive(hiqp_msgs::AddGeometricPrimitive::Request& req, 
                                                        hiqp_msgs::AddGeometricPrimitive::Response& res) {
  service_mutex_.lock();
  int retval = task_manager_.addGeometricPrimitive(
    req.name, req.type, req.frame_id, req.visible, req.color, req.parameters
  );
  res.success = (retval == 0 ? true : false);
  if (res.success) {
    hiqp::printHiqpInfo("Added geometric primitive of type '" + req.type + "' with name '" + req.name + "'.");
  }
  service_mutex_.unlock();
  return true;
}

/// \bug Removing primitive doesn't remove visualization
bool HiQPJointVelocityController::removeGeometricPrimitive(hiqp_msgs::RemoveGeometricPrimitive::Request& req, 
                                                           hiqp_msgs::RemoveGeometricPrimitive::Response& res) {
  service_mutex_.lock();
  res.success = false;
  if (task_manager_.removeGeometricPrimitive(req.name) == 0)
    res.success = true;

  if (res.success) {
    hiqp::printHiqpInfo("Removed primitive '" + req.name + "' successfully!");
  } else {
    hiqp::printHiqpInfo("Couldn't remove primitive '" + req.name + "'!");  
  }
  service_mutex_.unlock();
  return true;
}

bool HiQPJointVelocityController::removeAllGeometricPrimitives(hiqp_msgs::RemoveAllGeometricPrimitives::Request& req, 
                                                               hiqp_msgs::RemoveAllGeometricPrimitives::Response& res) {
  service_mutex_.lock();
  task_manager_.removeAllGeometricPrimitives();
  hiqp::printHiqpInfo("Removed all primitives successfully!");
  res.success = true;
  service_mutex_.unlock();
  return true;
}

bool HiQPJointVelocityController::listAllGeometricPrimitives(hiqp_msgs::ListAllGeometricPrimitives::Request& req, 
                                                             hiqp_msgs::ListAllGeometricPrimitives::Response& res) {
  service_mutex_.lock();
  task_manager_.listAllGeometricPrimitives();
  res.success = true;
  service_mutex_.unlock();
  return true;
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
  if (d.toSec() >= 1.0/rendering_publish_rate_) {
    last_rendering_update_ = now;
    task_manager_.renderPrimitives();
  }
}

void HiQPJointVelocityController::monitorTasks() {
  if (monitoring_active_) {
    ros::Time now = ros::Time::now();
    ros::Duration d = now - last_monitoring_update_;
    if (d.toSec() >= 1.0/monitoring_publish_rate_) {
      last_monitoring_update_ = now;
      std::vector<TaskMeasure> measures;
      task_manager_.getTaskMeasures(measures);

      hiqp_msgs::TaskMeasures msgs;
      msgs.stamp = now;
      for (auto&& measure : measures) {
        hiqp_msgs::TaskMeasure msg;
        msg.task_name = measure.task_name_;
        msg.e = std::vector<double>(measure.e_.data(), measure.e_.data() + measure.e_.rows() * measure.e_.cols());
        msg.de = std::vector<double>(measure.de_.data(), measure.de_.data() + measure.de_.rows() * measure.de_.cols());
        msg.pm = std::vector<double>(measure.pm_.data(), measure.pm_.data() + measure.pm_.rows() * measure.pm_.cols());
        msgs.task_measures.push_back(msg);
      }
      monitoring_pub_.publish(msgs);
    }
  }
}

void HiQPJointVelocityController::addAllTopicSubscriptions()
{
  topic_subscriber_.init( &task_manager_ );
  
  topic_subscriber_.addSubscription<hiqp_msgs::PoseWithName>(
    this->getControllerNodeHandle(), "/leap_hands/poses", 100
  );

  //topic_subscriber_.addSubscription<hiqp_msgs::Vector3d>(
  //  controller_nh_, "/yumi/hiqp_controllers/vector3d", 100
  //);
  
  //topic_subscriber_.addSubscription<hiqp_msgs::StringArray>(
  //  controller_nh_, "/yumi/hiqp_kinematics_controller/experiment_commands", 100
  //);
}




/// \todo Change add_primitive to set_primitive ros service
/// \todo Add list_all_primitives ros service
/// \todo Add show_primitive ros service
/// \todo Add hide_primitive ros service

/// \todo Add activate_controller ros service
/// \todo Add deactivate_controller ros service

/// \todo Add activate_task ros service
/// \todo Add deactivate_task ros service
/// \todo Add show_task ros service
/// \todo Add hide_task ros service
/// \todo Add monitor_task ros service
/// \todo Add unmonitor_task ros service

/// \todo Add activate_stage ros service
/// \todo Add deactivate_stage ros service
/// \todo Add monitor_stage ros service
/// \todo Add unmonitor_stage ros service

void HiQPJointVelocityController::advertiseAllServices()
{
  set_task_service_ = this->getControllerNodeHandle().advertiseService(
    "set_task", &HiQPJointVelocityController::setTask, this);

  remove_task_service_ = this->getControllerNodeHandle().advertiseService(
    "remove_task", &HiQPJointVelocityController::removeTask, this);

  remove_all_tasks_service_ = this->getControllerNodeHandle().advertiseService(
    "remove_all_tasks", &HiQPJointVelocityController::removeAllTasks, this);

  list_all_tasks_service_ = this->getControllerNodeHandle().advertiseService(
    "list_all_tasks", &HiQPJointVelocityController::listAllTasks, this);

  add_geomprim_service_ = this->getControllerNodeHandle().advertiseService(
    "add_primitive", &HiQPJointVelocityController::addGeometricPrimitive, this);

  remove_geomprim_service_ = this->getControllerNodeHandle().advertiseService(
    "remove_primitive", &HiQPJointVelocityController::removeGeometricPrimitive, this);

  remove_all_geomprims_service_ = this->getControllerNodeHandle().advertiseService(
    "remove_all_primitives", &HiQPJointVelocityController::removeAllGeometricPrimitives, this);

  list_all_geomprims_service_ = this->getControllerNodeHandle().advertiseService(
    "list_all_primitives", &HiQPJointVelocityController::listAllGeometricPrimitives, this);
}

// int HiQPJointVelocityController::loadDesiredSamplingTime()
// {
//   if (!this->getControllerNodeHandle().getParam("sampling_time", desired_sampling_time_)) {
//       ROS_ERROR_STREAM("In HiQPJointVelocityController: Call to getParam('sampling_time') in namespace '" 
//         << this->getControllerNodeHandle().getNamespace() 
//         << "' failed.");
//       return -1;
//   }
//   this->setDesiredSamplingTime(desired_sampling_time_);
//   //time_since_last_sampling_ = 0;
//   return 0;
// }

void HiQPJointVelocityController::loadRenderingParameters() {
  rendering_publish_rate_ = 1000; // defaults to 1 kHz
  if (!this->getControllerNodeHandle().getParam("visualization_publish_rate", rendering_publish_rate_)) {
    ROS_WARN("Couldn't find parameter 'visualization_publish_rate' on parameter server, defaulting to 1 kHz.");
  }
  last_rendering_update_ = ros::Time::now();
}

  /// \todo Task monitoring should publish an array of all task infos at each publication time step, rather than indeterministacally publishing single infos on the same topic
int HiQPJointVelocityController::loadAndSetupTaskMonitoring() {
  XmlRpc::XmlRpcValue task_monitoring;
  if (!this->getControllerNodeHandle().getParam("task_monitoring", task_monitoring)) {
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

  monitoring_pub_ = this->getControllerNodeHandle().advertise<hiqp_msgs::TaskMeasures>
  ("task_measures", 1);

  return 0;
}

/// \bug Having both, joint limits and avoidance tasks at the highest hierarchy level can cause an infeasible problem (e.g., via starting with yumi_hiqp_preload.yaml tasks)
void HiQPJointVelocityController::loadJointLimitsFromParamServer()
{
  XmlRpc::XmlRpcValue hiqp_preload_jnt_limits;
  if (!this->getControllerNodeHandle().getParam("hiqp_preload_jnt_limits", hiqp_preload_jnt_limits))
  {
    ROS_WARN_STREAM("No hiqp_preload_jnt_limits parameter found on "
      << "the parameter server. No joint limits were loaded!");
  } else {
    bool parsing_success = true;
    for (int i=0; i<hiqp_preload_jnt_limits.size(); ++i) {
      try {
        std::string link_frame = static_cast<std::string>(
          hiqp_preload_jnt_limits[i]["link_frame"] );

        XmlRpc::XmlRpcValue& limitations = 
        hiqp_preload_jnt_limits[i]["limitations"];

        std::vector<std::string> def_params;
        def_params.push_back("TDefJntLimits");
        def_params.push_back(link_frame);
        def_params.push_back( std::to_string(
          static_cast<double>(limitations[1]) ) );
        def_params.push_back( std::to_string(
          static_cast<double>(limitations[2]) ) );
        
        std::vector<std::string> dyn_params;
        dyn_params.push_back("TDynJntLimits");
        dyn_params.push_back( std::to_string(
          static_cast<double>(limitations[0]) ) );

        task_manager_.setTask(link_frame + "_jntlimits", 1, true, true,
          def_params, dyn_params, this->getRobotState());
      } catch (const XmlRpc::XmlRpcException& e) {
        ROS_WARN_STREAM("Error while loading "
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





void HiQPJointVelocityController::loadGeometricPrimitivesFromParamServer()
{
  XmlRpc::XmlRpcValue hiqp_preload_geometric_primitives;
  if (!this->getControllerNodeHandle().getParam("hiqp_preload_geometric_primitives", hiqp_preload_geometric_primitives)) {
    ROS_WARN_STREAM("No hiqp_preload_geometric_primitives parameter "
      << "found on the parameter server. No geometric primitives "
      << "were loaded!");
  } else {
    bool parsing_success = true;
    for (int i=0; i<hiqp_preload_geometric_primitives.size(); ++i) {
      try {
        std::string name = static_cast<std::string>(hiqp_preload_geometric_primitives[i]["name"] );
        std::string type = static_cast<std::string>(hiqp_preload_geometric_primitives[i]["type"] );
        std::string frame_id = static_cast<std::string>(hiqp_preload_geometric_primitives[i]["frame_id"] );
        bool visible = static_cast<bool>(hiqp_preload_geometric_primitives[i]["visible"] );

        XmlRpc::XmlRpcValue& color_xml = hiqp_preload_geometric_primitives[i]["color"];
        XmlRpc::XmlRpcValue& parameters_xml = hiqp_preload_geometric_primitives[i]["parameters"];

        std::vector<double> color;
        color.push_back( static_cast<double>(color_xml[0]) );
        color.push_back( static_cast<double>(color_xml[1]) );
        color.push_back( static_cast<double>(color_xml[2]) );
        color.push_back( static_cast<double>(color_xml[3]) );

        std::vector<double> parameters;
        for (int j=0; j<parameters_xml.size(); ++j){
          parameters.push_back(static_cast<double>(parameters_xml[j]));
        }

        task_manager_.addGeometricPrimitive(name, type, frame_id, visible, color, parameters);
      } catch (const XmlRpc::XmlRpcException& e) {
        ROS_WARN_STREAM("Error while loading "
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
  if (!this->getControllerNodeHandle().getParam("hiqp_preload_tasks", hiqp_preload_tasks)) {
    ROS_WARN_STREAM("No hiqp_preload_tasks parameter found on "
      << "the parameter server. No joint limits were loaded!");
  } else {
    bool parsing_success = true;
    for (int i=0; i<hiqp_preload_tasks.size(); ++i) {
      try {

        std::string name = static_cast<std::string>( hiqp_preload_tasks[i]["name"] );

        XmlRpc::XmlRpcValue& def_params_xml = hiqp_preload_tasks[i]["def_params"];
        std::vector<std::string> def_params;
        for (int j=0; j<def_params_xml.size(); ++j) {
          def_params.push_back( static_cast<std::string>( def_params_xml[j] ));
        }

        XmlRpc::XmlRpcValue& dyn_params_xml = hiqp_preload_tasks[i]["dyn_params"];
        std::vector<std::string> dyn_params;
        for (int j=0; j<dyn_params_xml.size(); ++j) {
          dyn_params.push_back( static_cast<std::string>( dyn_params_xml[j] ));
        }

        unsigned int priority = static_cast<int>( hiqp_preload_tasks[i]["priority"] );
        bool visible = static_cast<bool>( hiqp_preload_tasks[i]["visible"] );
        bool active = static_cast<bool>( hiqp_preload_tasks[i]["active"] );
        
        task_manager_.setTask(name, priority, visible, active, def_params, dyn_params, this->getRobotState());
      } catch (const XmlRpc::XmlRpcException& e) {
        ROS_WARN_STREAM("Error while loading "
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











} // namespace hiqp_ros


// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(hiqp_ros::HiQPJointVelocityController, controller_interface::ControllerBase)
