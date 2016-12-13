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

#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

#include <string>
#include <vector>
#include <mutex>
#include <memory>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>

#include <hiqp/robot_state.h>
#include <hiqp_ros/utilities.h>

namespace hiqp_ros {

  using hiqp::HiQPTimePoint;
  using hiqp::RobotState;
  using hiqp::RobotStatePtr;

  /*! \brief A base controller class whose controller type and hardware interface type are set as template parameters. This controller includes an internal KDL tree that is got from the robot_description on the parameter server. Also all claimed joint resources are read from the yaml file. Only joints specified in the yaml file are claimed, the others are not claimed but still read from.
   *  \author Marcus A Johansson */
  template <typename ControllerT, typename HardwareInterfaceT>
  class BaseController : public ControllerT {
  public:
    BaseController() = default;
    ~BaseController() noexcept = default;

    bool init(HardwareInterfaceT* hw, 
      ros::NodeHandle &controller_nh);

    void starting(const ros::Time& time) {}

    void update(const ros::Time& time, const ros::Duration& period);

    void stopping(const ros::Time& time) {}

    virtual void initialize() = 0;

    /*! \brief Implement this to set the output controls of this controller. Do not resize u! */
    virtual void computeControls(Eigen::VectorXd& u) = 0;

  protected:
    inline ros::NodeHandle& getControllerNodeHandle() { return controller_nh_; }
    inline unsigned int getNJoints() { return n_joints_; }
    inline RobotStatePtr getRobotState() { return robot_state_ptr_; }
    inline void setDesiredSamplingTime(double desired_sampling_time) { desired_sampling_time_ =  desired_sampling_time;}

  private:
    BaseController(const BaseController& other) = delete;
    BaseController(BaseController&& other) = delete;
    BaseController& operator=(const BaseController& other) = delete;
    BaseController& operator=(BaseController&& other) noexcept = delete;

    void loadDesiredSamplingTime();
    int loadUrdfToKdlTree();
    int loadJointsAndSetJointHandlesMap();
    void sampleJointValues();
    void setControls();

    typedef std::map<unsigned int, hardware_interface::JointHandle> JointHandleMap;

    RobotState                            robot_state_data_;
    RobotStatePtr                         robot_state_ptr_;
    HiQPTimePoint                         last_sampling_time_point_;
    double                                desired_sampling_time_;
    Eigen::VectorXd                       u_;

    ros::NodeHandle                       controller_nh_;
    HardwareInterfaceT*                   hardware_interface_;
    JointHandleMap                        joint_handles_map_;
    std::mutex                            handles_mutex_; 
    
    unsigned int                          n_joints_;

  };

  //////////////////////////////////////////////////////////////////////////////
  //
  //                      I M P L E M E N T A T I O N
  //
  //////////////////////////////////////////////////////////////////////////////

  template <typename ControllerT, typename HardwareInterfaceT>
  bool BaseController<ControllerT, HardwareInterfaceT>::init(HardwareInterfaceT* hw, ros::NodeHandle &controller_nh) {
    hardware_interface_ = hw;
    controller_nh_ = controller_nh;
    robot_state_ptr_.reset(&robot_state_data_);

    loadDesiredSamplingTime();
    loadUrdfToKdlTree();
    loadJointsAndSetJointHandlesMap();
    sampleJointValues();

    initialize();
    return true;
  }

  template <typename ControllerT, typename HardwareInterfaceT>
  void BaseController<ControllerT, HardwareInterfaceT>::update(const ros::Time& time, const ros::Duration& period) {
    HiQPTimePoint now(time.sec, time.nsec);
    double elapsed_time = (now-last_sampling_time_point_).toSec();
    if (elapsed_time*1000 >= desired_sampling_time_) {
      sampleJointValues();
      computeControls(u_);
      setControls();
    }
  }

  template <typename ControllerT, typename HardwareInterfaceT>
  void BaseController<ControllerT, HardwareInterfaceT>::loadDesiredSamplingTime() {
    desired_sampling_time_ = 1; // milliseconds. (defaults to 1kHz)
    if (!controller_nh_.getParam("sampling_time", desired_sampling_time_)) {
      ROS_WARN("Couldn't find parameter 'sampling_time' on the parameter server, defaulting to 1ms (1kHz).");
    }
    ros::Time t = ros::Time::now();
    last_sampling_time_point_.setTimePoint(t.sec, t.nsec);
  }

  template <typename ControllerT, typename HardwareInterfaceT>
  int BaseController<ControllerT, HardwareInterfaceT>::loadUrdfToKdlTree() {
    std::string full_parameter_path;
    std::string robot_urdf;
    if (controller_nh_.searchParam("robot_description", full_parameter_path)) {
      controller_nh_.getParam(full_parameter_path, robot_urdf);
      ROS_ASSERT(kdl_parser::treeFromString(robot_urdf, robot_state_data_.kdl_tree_));
      ROS_INFO("Loaded the robot's urdf model and initialized the KDL tree successfully");
    } else {
      ROS_ERROR("Could not find parameter 'robot_description' on the parameter server.");
      return -1;
    }
    return 0;
  }

  template <typename ControllerT, typename HardwareInterfaceT>
  int BaseController<ControllerT, HardwareInterfaceT>::loadJointsAndSetJointHandlesMap() {
    std::string param_name = "joints";
    std::vector< std::string > joint_names;
    if (!controller_nh_.getParam(param_name, joint_names)) {
      ROS_ERROR_STREAM("In ROSKinematicsController: Call to getParam('" 
        << param_name 
        << "') in namespace '" 
        << controller_nh_.getNamespace() 
        << "' failed.");
      return -1;
    }

    unsigned int n_joint_names = joint_names.size();
    n_joints_ = robot_state_data_.kdl_tree_.getNrOfJoints();
    if (n_joint_names > n_joints_) {
      ROS_ERROR_STREAM("In ROSKinematicsController: The .yaml file"
        << " includes more joint names than specified in the .urdf file."
        << " Could not successfully initialize controller. Aborting!\n");
      return -3;
    }

    std::vector<unsigned int> qnrs;
    hiqp::kdl_getAllQNrFromTree(robot_state_data_.kdl_tree_, qnrs);
    robot_state_data_.joint_handle_info_.clear();

    for (auto&& name : joint_names) {
      try {
        unsigned int q_nr = hiqp::kdl_getQNrFromJointName(robot_state_data_.kdl_tree_, name);
        //std::cout << "Joint found: '" << name << "', qnr: " << q_nr << "\n";
        joint_handles_map_.emplace(q_nr, hardware_interface_->getHandle(name));
        qnrs.erase(std::remove(qnrs.begin(), qnrs.end(), q_nr), qnrs.end());
        robot_state_data_.joint_handle_info_.push_back(hiqp::JointHandleInfo(q_nr, name, true, true));
      } catch (const hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return -2;
      }
      // catch (MAP INSERT FAIL EXCEPTION)
      // catch (HIQP Q_NR NOT AVAILABLE EXCEPTION)
    }

    for (auto&& qnr : qnrs) {
      std::string joint_name = hiqp::kdl_getJointNameFromQNr(robot_state_data_.kdl_tree_, qnr);
      hiqp::JointHandleInfo jhi(qnr, joint_name, true, false);
      robot_state_data_.joint_handle_info_.push_back(jhi);
    }

    std::cout << "Joint handle info:\n";
    for (auto&& jhi : robot_state_data_.joint_handle_info_) {
      std::cout << jhi.q_nr_ << ", " << jhi.joint_name_ << ", " << jhi.readable_ << ", " << jhi.writable_ << "\n";
    }

    robot_state_data_.kdl_jnt_array_vel_.resize(n_joints_);
    KDL::SetToZero(robot_state_data_.kdl_jnt_array_vel_.q);
    KDL::SetToZero(robot_state_data_.kdl_jnt_array_vel_.qdot);
    robot_state_data_.kdl_effort_.resize(n_joints_);
    KDL::SetToZero(robot_state_data_.kdl_effort_);
    u_ = Eigen::VectorXd::Zero(n_joints_);

    return 0;
  }

  template <typename ControllerT, typename HardwareInterfaceT>
  void BaseController<ControllerT, HardwareInterfaceT>::sampleJointValues() {
    KDL::JntArray& q = robot_state_data_.kdl_jnt_array_vel_.q;
    KDL::JntArray& qdot = robot_state_data_.kdl_jnt_array_vel_.qdot;
    KDL::JntArray& effort = robot_state_data_.kdl_effort_;
    handles_mutex_.lock();
      for (auto&& handle : joint_handles_map_) {
        q(handle.first) = handle.second.getPosition();
        qdot(handle.first) = handle.second.getVelocity();
        effort(handle.first) = handle.second.getEffort();
      }
      /* std::cerr<<"sampled joint positions: "<<q.data.transpose()<<std::endl; */
      /* std::cerr<<"sampled joint velocities: "<<qdot.data.transpose()<<std::endl; */
      /* std::cerr<<"sampled joint efforts: "<<effort.data.transpose()<<std::endl; */

      ros::Time t = ros::Time::now();
      robot_state_data_.sampling_time_point_.setTimePoint(t.sec, t.nsec);
      robot_state_data_.sampling_time_ = (robot_state_data_.sampling_time_point_ - last_sampling_time_point_).toSec();
      last_sampling_time_point_.setTimePoint(t.sec, t.nsec);
    handles_mutex_.unlock();
  }

  template <typename ControllerT, typename HardwareInterfaceT>
  void BaseController<ControllerT, HardwareInterfaceT>::setControls() {
    handles_mutex_.lock();
      for (auto&& handle : joint_handles_map_) {
        handle.second.setCommand(u_(handle.first));
      }
    handles_mutex_.unlock();
  }


} // namespace force_controllers

#endif // include guard
