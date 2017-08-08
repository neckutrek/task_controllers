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

#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>

#include <hiqp/robot_state.h>
#include <hiqp/tasks/tdef_geometric_projection.h>

#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>


namespace hiqp {
namespace tasks {

/*! \brief A task definition that positions geometric primitives relative to
 * each other through mutual geometric projection.
 *  \author Marcus A Johansson */
template <typename PrimitiveA, typename PrimitiveB>
class TDefGeometricProjectionWithNullspace : public TaskDefinition {
 public:
  inline TDefGeometricProjectionWithNullspace() : TaskDefinition() {}

  TDefGeometricProjectionWithNullspace(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
                          std::shared_ptr<Visualizer> visualizer);
  ~TDefGeometricProjectionWithNullspace() noexcept = default;

  int init(const std::vector<std::string>& parameters,
           RobotStatePtr robot_state);

  int update(RobotStatePtr robot_state);

  int monitor();

 private:
  TDefGeometricProjectionWithNullspace(const TDefGeometricProjectionWithNullspace& other) = delete;
  TDefGeometricProjectionWithNullspace(TDefGeometricProjectionWithNullspace&& other) = delete;
  TDefGeometricProjectionWithNullspace& operator=(const TDefGeometricProjectionWithNullspace& other) =
      delete;
  TDefGeometricProjectionWithNullspace& operator=(TDefGeometricProjectionWithNullspace&& other) noexcept =
      delete;

  int project(std::shared_ptr<PrimitiveA> first,
              std::shared_ptr<PrimitiveB> second);

  /// \brief This sets jacobian columns corresponding to non-writable joints to
  /// 0
  void maskJacobian(RobotStatePtr robot_state);

  /*! \brief Computes column number q_nr of the resulting jacobian for the
   *         vector (p2-p1), NOTE! p1 must be related to pose_a_ and p2 to
   *         pose_b_ !
   */
  KDL::Vector getVelocityJacobianForTwoPoints(const KDL::Vector& p1,
                                              const KDL::Vector& p2, int q_nr);

  KDL::Vector getVelocityJacobianForOnePoint(const KDL::Vector& p1, int q_nr);

  std::shared_ptr<KDL::TreeFkSolverPos_recursive> fk_solver_pos_;
  std::shared_ptr<KDL::TreeJntToJacSolver> fk_solver_jac_;

  std::shared_ptr<PrimitiveA> primitive_a_;
  KDL::Frame pose_a_;
  KDL::Jacobian jacobian_a_;

  std::shared_ptr<PrimitiveB> primitive_b_;
  KDL::Frame pose_b_;
  KDL::Jacobian jacobian_b_;
};

}  // namespace tasks

}  // namespace hiqp

#include <hiqp/tasks/tdef_geometric_projection_with_nullspace__impl.h>
