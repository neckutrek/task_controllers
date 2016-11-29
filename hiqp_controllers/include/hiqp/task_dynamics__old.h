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




/*!
 * \file   task_dynamics.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_TASK_DYNAMICS_H
#define HIQP_TASK_DYNAMICS_H

// HiQP Includes
#include <hiqp/hiqp_time_point.h>

// STL Includes
#include <vector>
#include <chrono>

// Eigen Includes
#include <Eigen/Dense>





namespace hiqp
{
	




class TaskFactory;

/*!
 * \class TaskDynamics
 * \brief Abstract base class for all task dynamics types.
 */ 
class TaskDynamics
{
public:

	TaskDynamics() {}
	~TaskDynamics() noexcept {}

	virtual int init
	(
		const HiQPTimePoint& sampling_time,
		const std::vector<std::string>& parameters,
    	const Eigen::VectorXd& e_initial,
   	 	const Eigen::VectorXd& e_final
	) = 0;

	virtual int apply
	(
		const HiQPTimePoint& sampling_time,
		const Eigen::VectorXd& e,
		const Eigen::MatrixXd& J,
		Eigen::VectorXd& e_dot_star
	) = 0;

	virtual int monitor() = 0;



	inline const std::string& getDynamicsTypeName()
	{ return dynamics_type_name_; }



protected:

    std::vector<double>             performance_measures_;





private:

	// No copying of this class is allowed !
	TaskDynamics(const TaskDynamics& other) = delete;
	TaskDynamics(TaskDynamics&& other) = delete;
	TaskDynamics& operator=(const TaskDynamics& other) = delete;
	TaskDynamics& operator=(TaskDynamics&& other) noexcept = delete;

	void setDynamicsTypeName(const std::string& name)
	{ dynamics_type_name_ = name; }

	std::string 					dynamics_type_name_;

	friend TaskFactory;


};








} // namespace hiqp






#endif // include guard