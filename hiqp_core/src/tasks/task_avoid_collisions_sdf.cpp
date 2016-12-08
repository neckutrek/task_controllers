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

#include <hiqp/tasks/task_avoid_collisions_sdf.h>
#include <hiqp/utilities.h>

#include <iostream>

namespace hiqp
{
  namespace tasks
  {
    //==================================================================================

    int TaskAvoidCollisionsSDF::init(const std::vector<std::string>& parameters,
				     RobotStatePtr robot_state,
				     unsigned int n_controls) {
      //   std::cout<<"printing parameters:"<<std::endl;
      //   for (int i=0; i<parameters.size();i++)
      //     {
      //       std::cout<<parameters.at(i)<<std::endl;
      // }
      int size = parameters.size();
      if (size < 2) {
	printHiqpWarning("TaskAvoidCollisionsSDF requires at least 2 parameters, got " 
			 + std::to_string(size) + "! Initialization failed!");
	return -2;
      }

      reset();

      //loop through all the geometric primitives intended for the obstacle avoidance and extract the pointers
      std::shared_ptr<GeometricPrimitiveMap> gpm = this->getGeometricPrimitiveMap();
      for (unsigned int i=1; i<size; i++)
	{
	  //make sure the primitive is either a point or a sphere - avoidance with other primitives is not currently implemented
	  if (gpm->getGeometricPrimitive<GeometricPoint>(parameters.at(i)) != nullptr)
	    {
	      std::shared_ptr<GeometricPoint> point=gpm->getGeometricPrimitive<GeometricPoint>(parameters.at(i));
	      //make sure the avoidance point is attached to the manipulator
	      if(kdl_getQNrFromLinkName(robot_state->kdl_tree_, point->getFrameId()) == -1)
		{
		  printHiqpWarning("TaskAvoidCollisionsSDF::init, avoidance point '" + parameters.at(i) + "' is not attached to the manipulator! Initialization failed.");
		  return -2; 
		}
	      point_list_.push_back(point);
	    }
	  else if (gpm->getGeometricPrimitive<GeometricSphere>(parameters.at(i)) != nullptr)
	    {
	      std::shared_ptr<GeometricSphere> sphere = gpm->getGeometricPrimitive<GeometricSphere>(parameters.at(i));
	      //make sure the avoidance sphere is attached to the manipulator
	      if(kdl_getQNrFromLinkName(robot_state->kdl_tree_, sphere->getFrameId()) == -1)
		{
		  printHiqpWarning("TaskAvoidCollisionsSDF::init, avoidance sphere '" + parameters.at(i) + "' is not attached to the manipulator! Initialization failed.");
		  return -2; 
		}
	      sphere_list_.push_back(sphere);
	    }
	  else
	    {
	      printHiqpWarning("TaskAvoidCollisionsSDF::init, couldn't find primitive '" + parameters.at(i) + "'! Initialization failed.");
	      return -2; 
	    }
	  n_dimensions_++;
	}

      e_.resize(n_dimensions_);
      e_.setZero();
      J_.resize(n_dimensions_, n_controls);
      J_.setZero();
      performance_measures_.resize(0);
      task_types_.insert(task_types_.begin(), n_dimensions_, -1); // -1 leq, 0 eq, 1 geq

      return 0;
    }
    //==================================================================================
    int TaskAvoidCollisionsSDF::update(RobotStatePtr robot_state) {
      // const KDL::JntArray &q = robot_state->kdl_jnt_array_vel_.q;
      // e_(0) = desired_configuration_ - q(joint_q_nr_);
      std::cout<<"Updating TaskAvoidCollisionsSDF"<<std::endl;

      return 0;
    }
    //==================================================================================
    int TaskAvoidCollisionsSDF::monitor() {
      return 0;
    }
    //==================================================================================
    void TaskAvoidCollisionsSDF::reset() {
      n_dimensions_=0;
      task_types_.clear();
      point_list_.clear();
      sphere_list_.clear();
    }
    //==================================================================================

  } // namespace tasks

} // namespace hiqp
