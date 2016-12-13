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

#ifndef HIQP_ROBOT_STATE_H
#define HIQP_ROBOT_STATE_H

#include <memory>
#include <kdl/tree.hpp>
#include <kdl/jntarrayvel.hpp>
#include <hiqp/hiqp_time_point.h>

namespace hiqp {

  /*! \brief Holds information on what q_nr (KDL) is associated with what joint name, and whether that joint's resource is readable/writable.
   *  \author Marcus A Johansson */
  struct JointHandleInfo {
    JointHandleInfo(unsigned int q_nr, const std::string& joint_name, bool readable, bool writable) 
     : q_nr_(q_nr), joint_name_(joint_name), readable_(readable), writable_(writable) {}
    unsigned int   q_nr_;
    std::string    joint_name_;
    bool           readable_;
    bool           writable_;
  };

  /*! \brief Holds the state of the robot (sampling time, kdl tree, joint positions and velocities) 
   *  \author Marcus A Johansson */
  struct RobotState {
    HiQPTimePoint                 sampling_time_point_;
    double                        sampling_time_;
    KDL::Tree                     kdl_tree_;
    KDL::JntArrayVel              kdl_jnt_array_vel_;
    KDL::JntArray                 kdl_effort_;
    std::vector<JointHandleInfo>  joint_handle_info_;

    /// \brief Returns whether the joint with qnr is writable or not
    inline bool isQNrWritable(unsigned int qnr) const {
      for (auto&& jhi : joint_handle_info_) {
        if (jhi.q_nr_ == qnr) 
          return jhi.writable_;
      }
      return false;
    }

    /// \brief Returns the total number of joints (including read-only joint resources)
    inline unsigned int getNumJoints() const {
      return kdl_tree_.getNrOfJoints();
    }

    /// \brief Returns the number of controllable joints (writable joint resources)
    inline unsigned int getNumControls() const {
      unsigned int n = 0;
      for (auto&& jhi : joint_handle_info_) {
        if (jhi.writable_) n++;
      }
      return n;
    }
  };

  /*! \brief A const pointer type to a robot state. Used to reference to the current robot state throught the framework.
   *  \author Marcus A Johansson */
  typedef std::shared_ptr<const RobotState> RobotStatePtr;

} // namespace hiqp

#endif // include guard