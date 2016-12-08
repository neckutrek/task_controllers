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

#ifndef HIQP_TASK_AVOID_COLLISIONS_SDF_H
#define HIQP_TASK_AVOID_COLLISIONS_SDF_H

#include <string>

#include <hiqp/hiqp_time_point.h>
#include <hiqp/task_definition.h>
#include <hiqp_collision_check/checker_base.h>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>

namespace hiqp
{
  namespace tasks
  {
    /*! \brief A struct holding Jacobian and end-effector point - used for forward kinematics.
     *  \author Robert Krug */  
    struct KinematicQuantities
    {
      std::string frame_id_;
      KDL::Jacobian ee_J_;
      KDL::Frame ee_pose_;
    };
    //==============================================================================================
    /*! \brief A task definition that allows avoidance of geometric primitives on the manipulator with the environment given as a SDF map.
     *  \author Robert Krug */  
    class TaskAvoidCollisionsSDF : public TaskDefinition {
    public:
    TaskAvoidCollisionsSDF(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
			   std::shared_ptr<Visualizer> visualizer)
      : TaskDefinition(geom_prim_map, visualizer) {}
      ~TaskAvoidCollisionsSDF() noexcept {}

      int init(const std::vector<std::string>& parameters,
	       RobotStatePtr robot_state,
	       unsigned int n_controls);

      int update(RobotStatePtr robot_state);

      int monitor();

    private:

      TaskAvoidCollisionsSDF(const TaskAvoidCollisionsSDF& other) = delete;
      TaskAvoidCollisionsSDF(TaskAvoidCollisionsSDF&& other) = delete;
      TaskAvoidCollisionsSDF& operator=(const TaskAvoidCollisionsSDF& other) = delete;
      TaskAvoidCollisionsSDF& operator=(TaskAvoidCollisionsSDF&& other) noexcept = delete;

      void reset();
      /*! This function computes the kinematic quantities for a primitive and clears the kin_q vector before computing*/
      int primitiveForwardKinematics(std::vector<KinematicQuantities>& kin_q_list, const std::shared_ptr<geometric_primitives::GeometricPrimitive>& primitive, RobotStatePtr robot_state)const;
      /*! Helper function which computes ee pose and Jacobian w.r.t. a given frame*/
      int forwardKinematics(KinematicQuantities& kin_q, const KDL::JntArray& q)const;

      std::shared_ptr<KDL::TreeFkSolverPos_recursive>  fk_solver_pos_;
      std::shared_ptr<KDL::TreeJntToJacSolver>         fk_solver_jac_;

      std::vector<std::shared_ptr<geometric_primitives::GeometricPrimitive> >    primitives_;
      std::string root_frame_id_;
      /*! Interface to the SDF map*/
      std::shared_ptr<hiqp::CollisionCheckerBase> collision_checker_;
    };

  } // namespace tasks

} // namespace hiqp


#endif // include guard
