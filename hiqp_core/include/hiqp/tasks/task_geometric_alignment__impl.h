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

#ifndef HIQP_TASK_GEOMETRIC_ALIGNMENT__IMPL_H
#define HIQP_TASK_GEOMETRIC_ALIGNMENT__IMPL_H

#include <sstream>
#include <iterator>

#include <hiqp/utilities.h>

namespace hiqp
{
namespace tasks
{

  template<typename PrimitiveA, typename PrimitiveB>
  TaskGeometricAlignment<PrimitiveA, PrimitiveB>::TaskGeometricAlignment(
    std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
    std::shared_ptr<Visualizer> visualizer)
  : TaskDefinition(geom_prim_map, visualizer) {}

  template<typename PrimitiveA, typename PrimitiveB>
  int TaskGeometricAlignment<PrimitiveA, PrimitiveB>::init(const std::vector<std::string>& parameters,
                                                           RobotStatePtr robot_state) {
    int parameters_size = parameters.size();
    if (parameters_size != 5) {
      printHiqpWarning("'" + getTaskName() + "': TDefGeomAlign takes 5 parameters, got " + std::to_string(parameters_size) + "! The task was not added!");
      return -1;
    }

    std::string prim_type1 = parameters.at(1);
    std::string prim_type2 = parameters.at(2);

    std::stringstream ss(parameters.at(3));
    std::vector<std::string> args(
      std::istream_iterator<std::string>{ss},
      std::istream_iterator<std::string>{});

    if (args.size() != 3) {
      printHiqpWarning("'" + getTaskName() + "': TDefGeomAlign's parameter nr.4 needs whitespace separation! The task was not added!");
      return -2;
    }

    unsigned int n_task_dimensions = 1;
    if (prim_type1.compare("frame") == 0 && prim_type2.compare("frame") == 0) {
      n_task_dimensions = 2;
    }
    
    unsigned int n_joints = robot_state->getNumJoints();
    e_.resize(n_task_dimensions);
    J_.resize(n_task_dimensions, n_joints);
    performance_measures_.resize(0);

    fk_solver_pos_ = std::make_shared<KDL::TreeFkSolverPos_recursive>(robot_state->kdl_tree_);
    fk_solver_jac_ = std::make_shared<KDL::TreeJntToJacSolver>(robot_state->kdl_tree_);

    std::shared_ptr<GeometricPrimitiveMap> gpm = this->getGeometricPrimitiveMap();

    primitive_a_ = gpm->getGeometricPrimitive<PrimitiveA>(args.at(0));
    primitive_b_ = gpm->getGeometricPrimitive<PrimitiveB>(args.at(2));

    gpm->addDependencyToPrimitive(args.at(0), this->getTaskName());
    gpm->addDependencyToPrimitive(args.at(2), this->getTaskName());

    int sign = 0;

    if (args.at(1).compare("<") == 0 || args.at(1).compare("<=") == 0) {
      sign = -1;
    } else if (args.at(1).compare("=") == 0 || args.at(1).compare("==") == 0) {
      sign = 0;
    } else if (args.at(1).compare(">") == 0 || args.at(1).compare(">=") == 0) {
      sign = 1;
    } else {
      return -3;
    }

    delta_ = std::stod( parameters.at(4) );
    task_types_.insert(task_types_.begin(), n_task_dimensions, sign);
    return 0;
  }

  template<typename PrimitiveA, typename PrimitiveB>
  int TaskGeometricAlignment<PrimitiveA, PrimitiveB>::update(RobotStatePtr robot_state) {
    int retval = 0;

    retval = fk_solver_pos_->JntToCart(robot_state->kdl_jnt_array_vel_.q, pose_a_, primitive_a_->getFrameId());
    if (retval != 0) {
      std::cerr << "In TaskGeometricAlignment::apply : Can't solve position "
        << "of link '" << primitive_a_->getFrameId() << "'" << " in the "
        << "KDL tree! KDL::TreeFkSolverPos_recursive::JntToCart return "
        << "error code '" << retval << "'\n";
      return -1;
    }

    retval = fk_solver_pos_->JntToCart(robot_state->kdl_jnt_array_vel_.q, pose_b_, primitive_b_->getFrameId());
    if (retval != 0) {
      std::cerr << "In TaskGeometricAlignment::apply : Can't solve position "
        << "of link '" << primitive_b_->getFrameId() << "'" << " in the "
        << "KDL tree! KDL::TreeFkSolverPos_recursive::JntToCart return "
        << "error code '" << retval << "'\n";
      return -2;
    }

    jacobian_a_.resize(robot_state->kdl_jnt_array_vel_.q.rows());
    retval = fk_solver_jac_->JntToJac(robot_state->kdl_jnt_array_vel_.q, jacobian_a_, primitive_a_->getFrameId());
    if (retval != 0) {
      std::cerr << "In TaskGeometricAlignment::apply : Can't solve jacobian "
        << "of link '" << primitive_a_->getFrameId() << "'" << " in the "
        << "KDL tree! KDL::TreeJntToJacSolver return error code "
        << "'" << retval << "'\n";
      return -3;
    }

    jacobian_b_.resize(robot_state->kdl_jnt_array_vel_.q.rows());
    retval = fk_solver_jac_->JntToJac(robot_state->kdl_jnt_array_vel_.q, jacobian_b_, primitive_b_->getFrameId());
    if (retval != 0) {
      std::cerr << "In TaskGeometricAlignment::apply : Can't solve jacobian "
        << "of link '" << primitive_b_->getFrameId() << "'" << " in the "
        << "KDL tree! KDL::TreeJntToJacSolver return error code "
        << "'" << retval << "'\n";
      return -4;
    }
    
    return align(primitive_a_, primitive_b_);
  }

  template<typename PrimitiveA, typename PrimitiveB>
  int TaskGeometricAlignment<PrimitiveA, PrimitiveB>::monitor() {
    return 0;
  }

  template<typename PrimitiveA, typename PrimitiveB>
  int TaskGeometricAlignment<PrimitiveA, PrimitiveB>::alignVectors
  (
    const KDL::Vector& v1, 
    const KDL::Vector v2
  )
  {
    double d = KDL::dot(v1, v2);

    e_(0) = d - std::cos(delta_);

    //std::cout << "e = " << e_(0) << "\n";

    KDL::Vector v = v1 * v2;    // v = v1 x v2

    for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr)
    {
      KDL::Vector Ja = jacobian_a_.getColumn(q_nr).rot;
      KDL::Vector Jb = jacobian_b_.getColumn(q_nr).rot;

      J_(0, q_nr) = KDL::dot( v, (Ja - Jb) );
    }

    return 0;
  }

} // namespace tasks

} // namespace hiqp

#endif // include guard