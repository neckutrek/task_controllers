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

#include <hiqp/utilities.h>

#include <hiqp/tasks/tdyn_jnt_limits.h>

namespace hiqp {
namespace tasks {

int TDynJntLimits::init(const std::vector<std::string>& parameters, RobotStatePtr robot_state, const Eigen::VectorXd& e_initial, const Eigen::VectorXd& e_dot_initial, const Eigen::VectorXd& e_final, const Eigen::VectorXd& e_dot_final) {
        assert((e_dot_initial.rows()==4) && (e_final.rows() == 4) && (e_dot_final.rows() == 4) );
  int size = parameters.size();
  if (size != 2) {
    printHiqpWarning("TDynJntLimits requires 2 parameters, got " +
                     std::to_string(size) + "! Initialization failed!");
    return -1;
  }
  Kp_=std::stod(parameters.at(1)); //P gain for joint limits

  //initialize the controller
  e_ddot_star_.resize(4);
  double dt=robot_state->sampling_time_;
  e_ddot_star_(0)= 1/dt*(Kp_*e_initial(0)+e_dot_initial(0));///upper joint limit
  e_ddot_star_(1)= 1/dt*(Kp_*e_initial(1)+e_dot_initial(1));///lower joint limit
  e_ddot_star_(2)= 1/dt*e_initial(2);//upper joint velocity limit
  e_ddot_star_(3)= 1/dt*e_initial(3);//lower joint velocity limit

  performance_measures_.resize(0);

      // //=============Debug======================
      // std::cerr<<"e_initial: "<<e_initial.transpose()<<std::endl;
      // std::cerr<<"e_dot_initial: "<<e_dot_initial.transpose()<<std::endl;
      // std::cerr<<"e_final: "<<e_final.transpose()<<std::endl;
      // std::cerr<<"e_dot_final: "<<e_dot_final.transpose()<<std::endl;
      // std::cerr<<"size: "<<size<<std::endl;
      // std::cerr<<"Kp_: "<<std::endl<<Kp_<<std::endl;
      // std::cerr<<"e_ddot_star: "<<e_ddot_star_.transpose()<<std::endl;
      // //===========End Debug====================

  return 0;
}

int TDynJntLimits::update(RobotStatePtr robot_state, const Eigen::VectorXd& e, const Eigen::VectorXd& e_dot, const Eigen::MatrixXd& J, const Eigen::MatrixXd& J_dot){
  double dt=robot_state->sampling_time_;
  e_ddot_star_(0)= 1/dt*(Kp_*e(0)+e_dot(0));///upper joint limit
  e_ddot_star_(1)= 1/dt*(Kp_*e(1)+e_dot(1));///lower joint limit
  e_ddot_star_(2)= 1/dt*e(2);//upper joint velocity limit
  e_ddot_star_(3)= 1/dt*e(3);//lower joint velocity limit

  //DEBUG===================================
  // std::cerr<<"dt: "<<dt<<std::endl;
  // std::cerr<<"e_ddot_star_: "<<e_ddot_star_.transpose()<<std::endl;
  // std::cerr<<"==========================================="<<std::endl;
  //DEBUG END ===============================
  return 0;
}

int TDynJntLimits::monitor() { return 0; }

}  // namespace tasks

}  // namespace hiqp
