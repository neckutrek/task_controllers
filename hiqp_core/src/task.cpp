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

#include <hiqp/task.h>
#include <hiqp/tasks/task_full_pose.h>
#include <hiqp/tasks/task_avoid_collisions_sdf.h>
#include <hiqp/tasks/task_jnt_config.h>
#include <hiqp/tasks/task_jnt_limits.h>
#include <hiqp/tasks/task_geometric_projection.h>
#include <hiqp/tasks/task_geometric_alignment.h>
#include <hiqp/tasks/dynamics_first_order.h>
#include <hiqp/tasks/dynamics_jnt_limits.h>
#include <hiqp/tasks/dynamics_minimal_jerk.h>
#include <hiqp/utilities.h>

#include <hiqp/geometric_primitives/geometric_point.h>
#include <hiqp/geometric_primitives/geometric_line.h>
#include <hiqp/geometric_primitives/geometric_cylinder.h>
#include <iomanip>

namespace hiqp {

  using tasks::TaskFullPose;
  using tasks::TaskJntConfig;
  using tasks::TaskJntLimits;
  using tasks::TaskGeometricProjection;
  using tasks::TaskGeometricAlignment;
  using tasks::TaskAvoidCollisionsSDF;
  using tasks::DynamicsFirstOrder;
  using tasks::DynamicsJntLimits;
  using tasks::DynamicsMinimalJerk;

  //using geometric_primitives::GeometricPoint;

  Task::Task(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
             std::shared_ptr<Visualizer> visualizer,
             int n_controls)
  : geom_prim_map_(geom_prim_map), visualizer_(visualizer), n_controls_(n_controls)
  {}

  int Task::init(const std::vector<std::string>& def_params,
                 const std::vector<std::string>& dyn_params,
                 RobotStatePtr robot_state) {
    if (def_params.size() <= 0) {
      printHiqpWarning("No (zero) task definition parameters found!");
      return -1;
    }
    if (dyn_params.size() <= 0) {
      printHiqpWarning("No (zero) task dynamics parameters found!");
      return -2;
    }

    if (constructDefinition(def_params) != 0) return -3;
    if (constructDynamics(dyn_params) != 0) return -4;

    def_->task_name_ = task_name_;
    def_->priority_ = priority_;
    def_->active_ = active_;
    def_->visible_ = visible_;

    dyn_->task_name_ = task_name_;
    dyn_->priority_ = priority_;
    dyn_->active_ = active_;
    dyn_->visible_ = visible_;

    if (def_->initialize(def_params, robot_state) != 0)
      return -5;

    //DEBUG ============================================
    // std::cerr<<"Task::init, initialized def_"<<std::endl;
    // std::cerr<<"e size: "<<def_->e_.size()<<std::endl;
    // std::cerr<<"J size: "<<def_->J_.rows()<<" "<<def_->J_.cols()<<std::endl;
    // std::cerr<<" types size: "<<def_->task_types_.size()<<std::endl;
    //DEBUG END ============================================

    if (dyn_->init(dyn_params, robot_state, def_->getInitialValue(), def_->getFinalValue(robot_state)) != 0)
      return -6;

    /// \bug Should enable consistench check - disabled now because the sdf - avoidance task ain't working with it
    // if (!checkConsistency(robot_state))
    //   return -7;

    // The task was successfully setup
    return 0;
  }

  int Task::update(RobotStatePtr robot_state)
  {
    if (!def_ || !dyn_) return -2;
    if (def_->update(robot_state) != 0) return -2;
    if (dyn_->update(robot_state, def_->e_, def_->J_) != 0) return -2;
   
    // DEBUG =============================================
    // std::cerr<<std::setprecision(2)<<"Update task '"<<getTaskName()<<"'"<<std::endl;
    // std::cerr<<"J_t:"<<std::endl<<def_->J_<<std::endl;
    // std::cerr<<"e: "<<def_->e_.transpose()<<std::endl;
    // std::cerr<<"signs: ";
    // for(int i=0; i<def_->task_types_.size();i++)
    //   std::cerr<<def_->task_types_[i]<<" ";

    // std::cerr<<std::endl<<"de*: "<<dyn_->e_dot_star_.transpose()<<std::endl;
    // DEBUG END ==========================================
    return 0;
  }

  int Task::constructDefinition(const std::vector<std::string>& def_params)
  {
    std::string type = def_params.at(0);

    if (type.compare("TDefFullPose") == 0) {
      def_ = std::make_shared<TaskFullPose>(geom_prim_map_, visualizer_);
    } else if (type.compare("TDefJntConfig") == 0) {
      def_ = std::make_shared<TaskJntConfig>(geom_prim_map_, visualizer_);
    } else if (type.compare("TDefAvoidCollisionsSDF") == 0) {
      def_ = std::make_shared<TaskAvoidCollisionsSDF>(geom_prim_map_, visualizer_);
    } else if (type.compare("TDefJntLimits") == 0) {
      def_ = std::make_shared<TaskJntLimits>(geom_prim_map_, visualizer_);
    } else if (type.compare("TDefGeomProj") == 0) {
      std::string prim_type1 = def_params.at(1);
      std::string prim_type2 = def_params.at(2);
      if (prim_type1.compare("point") == 0 && prim_type2.compare("point") == 0) {
        def_ = std::make_shared< TaskGeometricProjection<GeometricPoint, GeometricPoint> >(geom_prim_map_, visualizer_);
      } else if (prim_type1.compare("point") == 0 && prim_type2.compare("line") == 0) {
        def_ = std::make_shared< TaskGeometricProjection<GeometricPoint, GeometricLine> >(geom_prim_map_, visualizer_);
      } else if (prim_type1.compare("point") == 0 && prim_type2.compare("plane") == 0) {
        def_ = std::make_shared< TaskGeometricProjection<GeometricPoint, GeometricPlane> >(geom_prim_map_, visualizer_);
      } else if (prim_type1.compare("point") == 0 && prim_type2.compare("box") == 0) {
        def_ = std::make_shared< TaskGeometricProjection<GeometricPoint, GeometricBox> >(geom_prim_map_, visualizer_);
      } else if (prim_type1.compare("point") == 0 && prim_type2.compare("cylinder") == 0) {
        def_ = std::make_shared< TaskGeometricProjection<GeometricPoint, GeometricCylinder> >(geom_prim_map_, visualizer_);
      } else if (prim_type1.compare("point") == 0 && prim_type2.compare("sphere") == 0) {
        def_ = std::make_shared< TaskGeometricProjection<GeometricPoint, GeometricSphere> >(geom_prim_map_, visualizer_);
      } else if (prim_type1.compare("line") == 0 && prim_type2.compare("line") == 0) {
        def_ = std::make_shared< TaskGeometricProjection<GeometricLine, GeometricLine> >(geom_prim_map_, visualizer_);
      } else if (prim_type1.compare("sphere") == 0 && prim_type2.compare("plane") == 0) {
        def_ = std::make_shared< TaskGeometricProjection<GeometricSphere, GeometricPlane> >(geom_prim_map_, visualizer_);
      } else if (prim_type1.compare("sphere") == 0 && prim_type2.compare("sphere") == 0) {
        def_ = std::make_shared< TaskGeometricProjection<GeometricSphere, GeometricSphere> >(geom_prim_map_, visualizer_);
      } else if (prim_type1.compare("frame") == 0 && prim_type2.compare("frame") == 0) {
        def_ = std::make_shared< TaskGeometricProjection<GeometricFrame, GeometricFrame> >(geom_prim_map_, visualizer_);
      } else {
        printHiqpWarning("TDefGeomProj does not support primitive combination of types '" + prim_type1 + "' and '" + prim_type2 + "'!");
        return -1;
      }
    } else if (type.compare("TDefGeomAlign") == 0) {
      std::string prim_type1 = def_params.at(1);
      std::string prim_type2 = def_params.at(2);
      if (prim_type1.compare("line") == 0 && prim_type2.compare("line") == 0) {
        def_ = std::make_shared< TaskGeometricAlignment<GeometricLine, GeometricLine> >(geom_prim_map_, visualizer_);
      } else if (prim_type1.compare("line") == 0 && prim_type2.compare("plane") == 0) {
        def_ = std::make_shared< TaskGeometricAlignment<GeometricLine, GeometricPlane> >(geom_prim_map_, visualizer_);
      } else if (prim_type1.compare("line") == 0 && prim_type2.compare("cylinder") == 0) {
        def_ = std::make_shared< TaskGeometricAlignment<GeometricLine, GeometricCylinder> >(geom_prim_map_, visualizer_);
      } else if (prim_type1.compare("line") == 0 && prim_type2.compare("sphere") == 0) {
        def_ = std::make_shared< TaskGeometricAlignment<GeometricLine, GeometricSphere> >(geom_prim_map_, visualizer_);
      } else if (prim_type1.compare("frame") == 0 && prim_type2.compare("frame") == 0) {
        def_ = std::make_shared< TaskGeometricAlignment<GeometricFrame, GeometricFrame> >(geom_prim_map_, visualizer_);
      } else {
        printHiqpWarning("TDefGeomAlign does not support primitive combination of types '" + prim_type1 + "' and '" + prim_type2 + "'!");
        return -1;
      }
    } else {
      printHiqpWarning("The task definition type name '" + type + "' was not understood!");
      return -1;
    }

    return 0;
  }

  int Task::constructDynamics(const std::vector<std::string>& dyn_params)
  {
    std::string type = dyn_params.at(0);

    if (type.compare("TDynFirstOrder") == 0) {
      dyn_ = std::make_shared<DynamicsFirstOrder>(geom_prim_map_, visualizer_);
    } else if (type.compare("TDynJntLimits") == 0) {
      dyn_ = std::make_shared<DynamicsJntLimits>(geom_prim_map_, visualizer_);
    } else if (type.compare("TDynMinJerk") == 0) {
      dyn_ = std::make_shared<DynamicsMinimalJerk>(geom_prim_map_, visualizer_);
    } else {
      printHiqpWarning("The task dynamics type name '" + type + "' was not understood!");
      return -1;
    }

    return 0;
  }

  bool Task::checkConsistency(RobotStatePtr robot_state) {
    if (!def_) {
      printHiqpWarning("The task '" + task_name_ + "' is inconsistent after initialization. The task definition was not constructed properly.");
      return false;
    }

    if (!dyn_) {
      printHiqpWarning("The task '" + task_name_ + "' is inconsistent after initialization. The task dynamics was not constructed properly.");
      return false;
    }

    if (def_->e_.size() != def_->J_.rows()) {
      printHiqpWarning("The task '" + task_name_ + "' is inconsistent after initialization (dimension mismatch). " +
        "Size of task function (e_.size()) is " + std::to_string(def_->e_.size()) + ", " +
        "number of rows of task jacobian (J_.rows()) is " + std::to_string(def_->J_.rows()));
      return false;
    }

    if (def_->task_types_.size() != def_->J_.rows()) {
      printHiqpWarning("The task '" + task_name_ + "' is inconsistent after initialization (dimension mismatch). " +
        "Size of task types array (task_types_.size()) is " + std::to_string(def_->task_types_.size()) + ", " +
        "number of rows of task jacobian (J_.rows()) is " + std::to_string(def_->J_.rows()));
      return false;
    }

    if (dyn_->e_dot_star_.size() != def_->J_.rows()) {
      printHiqpWarning("The task '" + task_name_ + "' is inconsistent after initialization (dimension mismatch). " +
        "Size of desired task dynamics (e_dot_star_.size()) is " + std::to_string(dyn_->e_dot_star_.size()) + ", " +
        "number of rows of task jacobian (J_.rows()) is " + std::to_string(def_->J_.rows()));
      return false;
    }

    if (def_->J_.cols() != robot_state->getNumJoints()) {
      printHiqpWarning("The task '" + task_name_ + "' is inconsistent after initialization (dimension mismatch). " +
        "Number of columns of task jacobian (J_.cols()) is " + std::to_string(def_->J_.cols()) + ", " +
        "total number of joints in the robot is " + std::to_string(robot_state->getNumJoints()));
      return false;
    }

    // the task definition and dynamics are consistent
    return true;
  }





} // namespace hiqp
