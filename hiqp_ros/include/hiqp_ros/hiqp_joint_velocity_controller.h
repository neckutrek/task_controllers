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
#include <mutex>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <hiqp/task_manager.h>
#include <hiqp/hiqp_time_point.h>

#include <hiqp_ros/base_controller.h>
#include <hiqp_ros/ros_visualizer.h>
#include <hiqp_ros/ros_topic_subscriber.h>

#include <hiqp_msgs/SetTask.h>
#include <hiqp_msgs/UpdateTask.h>
#include <hiqp_msgs/RemoveTask.h>
#include <hiqp_msgs/RemoveAllTasks.h>
#include <hiqp_msgs/ListAllTasks.h>
#include <hiqp_msgs/AddGeometricPrimitive.h>
#include <hiqp_msgs/RemoveGeometricPrimitive.h>
#include <hiqp_msgs/RemoveAllGeometricPrimitives.h>
#include <hiqp_msgs/ListAllGeometricPrimitives.h>

#include <fstream>

namespace hiqp_ros
{
  typedef controller_interface::Controller<hardware_interface::VelocityJointInterface> JointVelocityController;
  typedef hardware_interface::VelocityJointInterface JointVelocityInterface;

  /*! \brief A joint velocity controller that provides full access to the HiQP control framework
   *  \author Marcus A Johansson */  
  class HiQPJointVelocityController : public BaseController<JointVelocityController, JointVelocityInterface> {
  public:
    HiQPJointVelocityController();
    ~HiQPJointVelocityController() noexcept;
    
    void initialize();
    void computeControls(Eigen::VectorXd& u);

  private:
    HiQPJointVelocityController(const HiQPJointVelocityController& other) = delete;
    HiQPJointVelocityController(HiQPJointVelocityController&& other) = delete;
    HiQPJointVelocityController& operator=(const HiQPJointVelocityController& other) = delete;
    HiQPJointVelocityController& operator=(HiQPJointVelocityController&& other) noexcept = delete;

    void monitorTasks();
    void renderPrimitives();

    void loadRenderingParameters();
    int loadAndSetupTaskMonitoring();
    void advertiseAllServices();
    void addAllTopicSubscriptions();
    void loadJointLimitsFromParamServer();
    void loadGeometricPrimitivesFromParamServer();
    void loadTasksFromParamServer();

    bool setTask(hiqp_msgs::SetTask::Request& req, hiqp_msgs::SetTask::Response& res);
    bool removeTask(hiqp_msgs::RemoveTask::Request& req, hiqp_msgs::RemoveTask::Response& res);
    bool removeAllTasks(hiqp_msgs::RemoveAllTasks::Request& req, hiqp_msgs::RemoveAllTasks::Response& res);
    bool listAllTasks(hiqp_msgs::ListAllTasks::Request& req, hiqp_msgs::ListAllTasks::Response& res);
    bool addGeometricPrimitive(hiqp_msgs::AddGeometricPrimitive::Request& req, hiqp_msgs::AddGeometricPrimitive::Response& res);
    bool removeGeometricPrimitive(hiqp_msgs::RemoveGeometricPrimitive::Request& req, hiqp_msgs::RemoveGeometricPrimitive::Response& res);
    bool removeAllGeometricPrimitives(hiqp_msgs::RemoveAllGeometricPrimitives::Request& req, hiqp_msgs::RemoveAllGeometricPrimitives::Response& res);
    bool listAllGeometricPrimitives(hiqp_msgs::ListAllGeometricPrimitives::Request& req, hiqp_msgs::ListAllGeometricPrimitives::Response& res);

    typedef std::map<unsigned int, hardware_interface::JointHandle > JointHandleMap;

    bool                                              is_active_;

    //double                                            desired_visualization_publishing_time_;
    //hiqp::HiQPTimePoint                               last_visualization_publishing_time_;
    //double                                            time_since_last_visualization_publishing_;

    bool                                              monitoring_active_;
    double                                            monitoring_publish_rate_;
    ros::Time                                         last_monitoring_update_;

    double                                            rendering_publish_rate_;
    ros::Time                                         last_rendering_update_;

    ros::Publisher                                    monitoring_pub_;

    ROSTopicSubscriber                                topic_subscriber_;

    ros::ServiceServer                                set_task_service_;
    ros::ServiceServer                                update_task_service_;
    ros::ServiceServer                                remove_task_service_;
    ros::ServiceServer                                remove_all_tasks_service_;
    ros::ServiceServer                                list_all_tasks_service_;
    ros::ServiceServer                                add_geomprim_service_;
    ros::ServiceServer                                remove_geomprim_service_;
    ros::ServiceServer                                remove_all_geomprims_service_;
    ros::ServiceServer                                list_all_geomprims_service_;

    std::mutex                                        service_mutex_; 

    std::shared_ptr<Visualizer>                       visualizer_;
    ROSVisualizer                                     ros_visualizer_;
    hiqp::TaskManager                                 task_manager_;

  };

} // namespace hiqp

#endif // include guard
