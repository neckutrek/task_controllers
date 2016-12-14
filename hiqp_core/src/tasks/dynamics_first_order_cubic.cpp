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

#include <hiqp/tasks/dynamics_first_order_cubic.h>

namespace hiqp
{
  namespace tasks
  {

    int DynamicsFirstOrderCubic::init(const std::vector<std::string>& parameters,
				      RobotStatePtr robot_state,
				      const Eigen::VectorXd& e_initial,
				      const Eigen::VectorXd& e_final) {
    int size = parameters.size();
      if (size != 2)
	{
	  printHiqpWarning("TDynFirstOrderCubic requires 2 parameters, got " 
			   + std::to_string(size) + "! Initialization failed!");
	  return -1;
	}

      lambda_ = std::stod( parameters.at(1) );

      e_dot_star_.resize(e_initial.rows());
      performance_measures_.resize(e_initial.rows());
      
      return 0;
    }

    int DynamicsFirstOrderCubic::update(RobotStatePtr robot_state,
					const Eigen::VectorXd& e,
					const Eigen::MatrixXd& J) {

      e_dot_star_.resize(e.size());
      for (unsigned int i=0; i<e_dot_star_.size();i++)
	e_dot_star_(i)=pow(e(i),3);

      e_dot_star_ = -lambda_ * e_dot_star_;
      //std::cout << "e_dot_star = " << e_dot_star << "\n\n";
      return 0;
    }

    int DynamicsFirstOrderCubic::monitor() {
      return 0;
    }

  } // namespace tasks

} // namespace hiqp
