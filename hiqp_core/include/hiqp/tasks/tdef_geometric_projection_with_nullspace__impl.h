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

#include <iterator>
#include <sstream>

#include <hiqp/utilities.h>

namespace hiqp {
namespace tasks {

template <typename PrimitiveA, typename PrimitiveB>
TDefGeometricProjectionWithNullspace<PrimitiveA, PrimitiveB>::TDefGeometricProjectionWithNullspace(
    std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
    std::shared_ptr<Visualizer> visualizer)
    : TaskDefinition(geom_prim_map, visualizer) {}

template <typename PrimitiveA, typename PrimitiveB>
int TDefGeometricProjectionWithNullspace<PrimitiveA, PrimitiveB>::init(
    const std::vector<std::string>& parameters, RobotStatePtr robot_state) {
  int parameters_size = parameters.size();
  if (parameters_size != 4) {
    printHiqpWarning(
        "'" + getTaskName() + "': TDefGeomProj takes 4 parameters, got " +
        std::to_string(parameters_size) + "! The task was not added!");
    return -1;
  }

  // Used for debugging
  // if (parameters.at(2).compare("box") == 0) {
  //   getGeometricPrimitiveMap()->addGeometricPrimitive("box_center", "point",
  //   "world", true, {0, 0, 1, 1}, {0, 0, 0});
  //   getGeometricPrimitiveMap()->addGeometricPrimitive("box_proj", "point",
  //   "world", true, {0, 0, 1, 1}, {0, 0, 0});
  //   getGeometricPrimitiveMap()->addGeometricPrimitive("box_line", "line",
  //   "world", true, {0, 0, 1, 1}, {0, 0, 0, 0, 0, 0});
  // }

  std::stringstream ss(parameters.at(3));
  std::vector<std::string> args(std::istream_iterator<std::string>{ss},
                                std::istream_iterator<std::string>{});

  if (args.size() != 3) {
    printHiqpWarning("'" + getTaskName() +
                     "': TDefGeomProj's parameter nr.4 needs whitespace "
                     "separation! The task was not added!");
    return -2;
  }

  unsigned int n_joints = robot_state->getNumJoints();
  // e_.resize(1);
  // J_.resize(2, n_joints);

  e_.resize(3);
  J_.resize(3, n_joints);

//  if(parameters.at(2).compare("plane") == 0){
//  }
  // e_.resize(3);
  // J_.resize(3, n_joints);

  performance_measures_.resize(0);

  fk_solver_pos_ =
      std::make_shared<KDL::TreeFkSolverPos_recursive>(robot_state->kdl_tree_);
  fk_solver_jac_ =
      std::make_shared<KDL::TreeJntToJacSolver>(robot_state->kdl_tree_);

  std::shared_ptr<GeometricPrimitiveMap> gpm = this->getGeometricPrimitiveMap();

  primitive_a_ = gpm->getGeometricPrimitive<PrimitiveA>(args.at(0));
  if (primitive_a_ == nullptr) {
    printHiqpWarning(
        "In TDefGeometricProjectionWithNullspace::init(), couldn't find primitive with name "
        "'" +
        args.at(0) + "'. Unable to create task!");
    return -3;
  }

  primitive_b_ = gpm->getGeometricPrimitive<PrimitiveB>(args.at(2));
  if (primitive_b_ == nullptr) {
    printHiqpWarning(
        "In TDefGeometricProjectionWithNullspace::init(), couldn't find primitive with name "
        "'" +
        args.at(2) + "'. Unable to create task!");
    return -3;
  }

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
    return -4;
  }

  task_types_.clear();
  task_types_.resize(3);
  // task_types_.resize(3);
  task_types_.at(0) = sign;
  task_types_.at(1) = sign;
  task_types_.at(2) = sign;

  // task_types_.at(2) = sign;

  return 0;
}

template <typename PrimitiveA, typename PrimitiveB>
int TDefGeometricProjectionWithNullspace<PrimitiveA, PrimitiveB>::update(
    RobotStatePtr robot_state) {
  int retval = 0;

  retval = fk_solver_pos_->JntToCart(robot_state->kdl_jnt_array_vel_.q, pose_a_,
                                     primitive_a_->getFrameId());
  if (retval != 0) {
    std::cerr << "In TDefGeometricProjectionWithNullspace::apply : Can't solve position "
              << "of link '" << primitive_a_->getFrameId() << "'"
              << " in the "
              << "KDL tree! KDL::TreeFkSolverPos_recursive::JntToCart return "
              << "error code '" << retval << "'\n";
    return -1;
  }

  retval = fk_solver_pos_->JntToCart(robot_state->kdl_jnt_array_vel_.q, pose_b_,
                                     primitive_b_->getFrameId());
  if (retval != 0) {
    std::cerr << "In TDefGeometricProjectionWithNullspace::apply : Can't solve position "
              << "of link '" << primitive_b_->getFrameId() << "'"
              << " in the "
              << "KDL tree! KDL::TreeFkSolverPos_recursive::JntToCart return "
              << "error code '" << retval << "'\n";
    return -2;
  }

  jacobian_a_.resize(robot_state->kdl_jnt_array_vel_.q.rows());
  retval = fk_solver_jac_->JntToJac(robot_state->kdl_jnt_array_vel_.q,
                                    jacobian_a_, primitive_a_->getFrameId());
  if (retval != 0) {
    std::cerr << "In TDefGeometricProjectionWithNullspace::apply : Can't solve jacobian "
              << "of link '" << primitive_a_->getFrameId() << "'"
              << " in the "
              << "KDL tree! KDL::TreeJntToJacSolver return error code "
              << "'" << retval << "'\n";
    return -3;
  }

  jacobian_b_.resize(robot_state->kdl_jnt_array_vel_.q.rows());
  retval = fk_solver_jac_->JntToJac(robot_state->kdl_jnt_array_vel_.q,
                                    jacobian_b_, primitive_b_->getFrameId());
  if (retval != 0) {
    std::cerr << "In TDefGeometricProjectionWithNullspace::apply : Can't solve jacobian "
              << "of link '" << primitive_b_->getFrameId() << "'"
              << " in the "
              << "KDL tree! KDL::TreeJntToJacSolver return error code "
              << "'" << retval << "'\n";
    return -4;
  }

  project(primitive_a_, primitive_b_);
  maskJacobian(robot_state);
  return 0;
}

template <typename PrimitiveA, typename PrimitiveB>
int TDefGeometricProjectionWithNullspace<PrimitiveA, PrimitiveB>::monitor() {
  return 0;
}

template <typename PrimitiveA, typename PrimitiveB>
KDL::Vector TDefGeometricProjectionWithNullspace<PrimitiveA, PrimitiveB>::
    getVelocityJacobianForTwoPoints(const KDL::Vector& p1,
                                    const KDL::Vector& p2, int q_nr) {
  KDL::Twist Ja = jacobian_a_.getColumn(q_nr);
  KDL::Twist Jb = jacobian_b_.getColumn(q_nr);
  KDL::Vector Jp1 = Ja.rot * p1;
  KDL::Vector Jp2 = Jb.rot * p2;

  return (Jb.vel + Jp2 - (Ja.vel + Jp1));
}

template <typename PrimitiveA, typename PrimitiveB>
KDL::Vector TDefGeometricProjectionWithNullspace<PrimitiveA, PrimitiveB>::
    getVelocityJacobianForOnePoint(const KDL::Vector& p1, int q_nr) {
  KDL::Twist Ja = jacobian_a_.getColumn(q_nr);
  KDL::Vector Jp1 = Ja.rot * p1;

  return (Ja.vel + Jp1);
}


template <typename PrimitiveA, typename PrimitiveB>
void TDefGeometricProjectionWithNullspace<PrimitiveA, PrimitiveB>::maskJacobian(
    RobotStatePtr robot_state) {
  for (unsigned int c = 0; c < robot_state->getNumJoints(); ++c) {
    if (!robot_state->isQNrWritable(c)) J_.col(c).setZero();
  }
}

}  // namespace tasks

}  // namespace hiqp