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

#ifndef HIQP_TASK_GEOMETRIC_ALIGNMENT_H
#define HIQP_TASK_GEOMETRIC_ALIGNMENT_H

#include <string>
#include <vector>

#include <hiqp/robot_state.h>
#include <hiqp/task_definition.h>

#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>

namespace hiqp
{
namespace tasks
{

  /*! \brief A task definition that rotates primitives to align with each other.
   *  \author Marcus A Johansson */  
  template<typename PrimitiveA, typename PrimitiveB>
  class TaskGeometricAlignment : public TaskDefinition {
  public:
    TaskGeometricAlignment(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
                           std::shared_ptr<Visualizer> visualizer);
    ~TaskGeometricAlignment() noexcept = default;

    int init(const std::vector<std::string>& parameters,
             RobotStatePtr robot_state);

    int update(RobotStatePtr robot_state);

    int monitor();

  private:
    TaskGeometricAlignment(const TaskGeometricAlignment& other) = delete;
    TaskGeometricAlignment(TaskGeometricAlignment&& other) = delete;
    TaskGeometricAlignment& operator=(const TaskGeometricAlignment& other) = delete;
    TaskGeometricAlignment& operator=(TaskGeometricAlignment&& other) noexcept = delete;

    int align(std::shared_ptr<PrimitiveA> first, std::shared_ptr<PrimitiveB> second);
    int alignVectors(const KDL::Vector& v1, const KDL::Vector v2);

    std::shared_ptr<KDL::TreeFkSolverPos_recursive>  fk_solver_pos_;
    std::shared_ptr<KDL::TreeJntToJacSolver>         fk_solver_jac_;

    std::shared_ptr<PrimitiveA>  primitive_a_;
    KDL::Frame                   pose_a_;
    KDL::Jacobian                jacobian_a_;

    std::shared_ptr<PrimitiveB>  primitive_b_;
    KDL::Frame                   pose_b_;
    KDL::Jacobian                jacobian_b_;

    double                       delta_; // the angular error margin
  };

} // namespace tasks

} // namespace hiqp

#include <hiqp/tasks/task_geometric_alignment__impl.h>

#endif // include guard