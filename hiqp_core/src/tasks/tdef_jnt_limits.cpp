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

#include <hiqp/tasks/tdef_jnt_limits.h>
#include <hiqp/utilities.h>

#include <iostream>

namespace hiqp {
  namespace tasks {

    int TDefJntLimits::init(const std::vector<std::string>& parameters,
			    RobotStatePtr robot_state) {
      int size = parameters.size();
      if (size != 6) {
	printHiqpWarning("TDefJntLimits requires 6 parameter, got " +
			 std::to_string(size) + "! Initialization failed!");
	return -1;
      }

      link_frame_name_ = parameters.at(1);
      link_frame_q_nr_ =
	kdl_getQNrFromLinkName(robot_state->kdl_tree_, link_frame_name_);

      if (link_frame_q_nr_ < 0) {
	printHiqpWarning("TDefJntLimits::init, couldn't find joint '" +
			 link_frame_name_ + "'! Initialization failed.");
	return -2;
      }

      if (!robot_state->isQNrWritable(link_frame_q_nr_)) {
	printHiqpWarning("TDefJntLimits::init, the joint '" + link_frame_name_ +
			 "' is not writable! Initialization failed.");
	return -3;
      }

      unsigned int n_joints = robot_state->getNumJoints();
       double q = robot_state->kdl_jnt_array_vel_.q(link_frame_q_nr_);
      double q_dot = robot_state->kdl_jnt_array_vel_.qdot(link_frame_q_nr_);
      q_lb_= std::stod(parameters.at(2));
      q_ub_= std::stod(parameters.at(3));
      dq_max_= std::stod(parameters.at(4));
      inf_zone_= std::stod(parameters.at(5));
      assert(q_ub_ > q_lb_);
      assert(dq_max_ > 0.0);
      assert((inf_zone_ > 0) && (inf_zone_ < (q_ub_ - q_lb_)));

      //DEBUG===================================
      std::cerr<<"q_lb: "<<q_lb_<<std::endl;
      std::cerr<<"q_ub: "<<q_ub_<<std::endl;
      std::cerr<<"dq_max: "<<dq_max_<<std::endl;
      std::cerr<<"inf zone: "<<inf_zone_<<std::endl;
      //DEBUG END===============================

      
      e_.resize(4);
      e_dot_.resize(4);
      J_.resize(4, n_joints);
      J_dot_.resize(4, n_joints);
      performance_measures_.resize(0);

      task_types_.resize(4);
      task_types_.at(0) = -1;   // < lower joint limit 
      task_types_.at(1) = 1;  // > upper joint limit
      task_types_.at(2) = -1;   // < lower joint velocity limit
      task_types_.at(3) = 1;  // > upper joint velocity limit

      // Jacobian entries are -1 at column link_frame_q_nr_ and 0 otherwise
      for (int i = 0; i < 4; ++i) {
	for (int j = 0; j < n_joints; ++j) {
	  J_(i, j) = (j == link_frame_q_nr_ ? -1 : 0);
	}
      }

      J_dot_.setZero();
      
      //Initialize e, e_dot
      e_(0) = q_lb_ - q;
      e_(1) = q_ub_ - q;
      e_dot_(0) = -q_dot;
      e_dot_(1) = -q_dot;
      if(abs(e_(0)) > inf_zone_){
	e_(0)=0.0;
	e_dot_(1)=0.0;
	J_(0,link_frame_q_nr_)=0.0;
      }
      if(abs(e_(1)) > inf_zone_){
	e_(1)=0.0;
	e_dot_(1)=0.0;
        J_(1,link_frame_q_nr_)=0.0;
      }
      
      e_(2) = -dq_max_ - q_dot;
      e_(3) = dq_max_ - q_dot;
      //the error derivatives for joint velocity limits are actually not defined as we are controlling in acceleration
      e_dot_(2) = 0.0;
      e_dot_(3) = 0.0;
      
      return 0;
    }

    int TDefJntLimits::update(RobotStatePtr robot_state) {
      double q = robot_state->kdl_jnt_array_vel_.q(link_frame_q_nr_);
      double q_dot = robot_state->kdl_jnt_array_vel_.qdot(link_frame_q_nr_);
     
      if(abs(e_(0)) > inf_zone_){
	e_(0)=0.0;
	e_dot_(1)=0.0;
	J_(0,link_frame_q_nr_)=0.0;
      }
      else{
	e_(0) = q_lb_ - q;
	e_dot_(0) = -q_dot;
	J_(0,link_frame_q_nr_)=-1.0;
      }
      if(abs(e_(1)) > inf_zone_){
	e_(1)=0.0;
	e_dot_(1)=0.0;
	J_(1,link_frame_q_nr_)=0.0;
      }
      else{
	e_(1) = q_ub_ - q;
	e_dot_(1) = -q_dot;
        J_(1,link_frame_q_nr_)=-1.0;
      }
      
      e_(2) = -dq_max_ - q_dot;
      e_(3) = dq_max_ - q_dot;
      //the error derivatives for joint velocity limits are actually not defined as we are controlling in acceleration
      e_dot_(2) = 0.0;
      e_dot_(3) = 0.0;
      return 0;
    }

    int TDefJntLimits::monitor() { return 0; }

  }  // namespace tasks

}  // namespace hiqp
