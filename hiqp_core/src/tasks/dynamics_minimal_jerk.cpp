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

#include <hiqp/tasks/dynamics_minimal_jerk.h>

namespace hiqp
{
namespace tasks
{

  int DynamicsMinimalJerk::init(const std::vector<std::string>& parameters,
                                RobotStatePtr robot_state,
                                const Eigen::VectorXd& e_initial,
                                const Eigen::VectorXd& e_final) {
    int size = parameters.size();
    if (size != 3) {
      printHiqpWarning("TDynMinimalJerk requires 3 parameters, got " 
        + std::to_string(size) + "! Initialization failed!");
      return -1;
    }

    performance_measures_.resize(e_initial.rows());
    e_dot_star_.resize(e_initial.rows());

    time_start_ = robot_state->sampling_time_point_;

    total_duration_ = std::stod( parameters.at(1) );
    gain_ = std::stod( parameters.at(2) );

    f_ = 30 / total_duration_;

    e_initial_ = e_initial;
    e_final_ = e_final;
    e_diff_ = e_final - e_initial;

    return 0;
  }

  int DynamicsMinimalJerk::update(RobotStatePtr robot_state,
                                  const Eigen::VectorXd& e,
                                  const Eigen::MatrixXd& J) {
    e_dot_star_.resize(e.size());
    double tau = (robot_state->sampling_time_point_ - time_start_).toSec() / total_duration_;

    if (tau > 1) {
      e_dot_star_ = 0*e;
    } else {
      double T = 10*tau*tau*tau - 15*tau*tau*tau*tau + 6*tau*tau*tau*tau*tau;
      double t = f_ * (tau*tau - 2*tau*tau*tau + tau*tau*tau*tau);

      Eigen::VectorXd e_star = e_initial_ + e_diff_ * T;

      e_dot_star_ = e_diff_ * t - gain_ * (e - e_star); // minimal jerk + first order
    }

    return 0;
  }

  int DynamicsMinimalJerk::monitor() {
    return 0;
  }

} // namespace tasks

} // namespace hiqp
