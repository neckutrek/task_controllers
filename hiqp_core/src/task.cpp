
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

#include <hiqp/tasks/tdef_full_pose.h>
#include <hiqp/tasks/tdef_geometric_alignment.h>
#include <hiqp/tasks/tdef_geometric_projection.h>
#include <hiqp/tasks/tdef_geometric_projection_with_nullspace.h>
#include <hiqp/tasks/tdef_jnt_config.h>
#include <hiqp/tasks/tdef_jnt_limits.h>
#include <hiqp/tasks/tdef_meta_task.h>

#include <hiqp/tasks/tdyn_cubic.h>
#include <hiqp/tasks/tdyn_hyper_sin.h>
#include <hiqp/tasks/tdyn_jnt_limits.h>
#include <hiqp/tasks/tdyn_linear.h>
#include <hiqp/tasks/tdyn_minimal_jerk.h>

#include <hiqp/utilities.h>

#include <hiqp/geometric_primitives/geometric_cylinder.h>
#include <hiqp/geometric_primitives/geometric_line.h>
#include <hiqp/geometric_primitives/geometric_point.h>
#include <iomanip>

namespace hiqp {

using tasks::TDefFullPose;
using tasks::TDefGeometricAlignment;
using tasks::TDefGeometricProjection;
using tasks::TDefGeometricProjectionWithNullspace;
using tasks::TDefJntConfig;
using tasks::TDefJntLimits;
using tasks::TDefMetaTask;

using tasks::TDynLinear;
using tasks::TDynCubic;
using tasks::TDynHyperSin;
using tasks::TDynJntLimits;
using tasks::TDynMinimalJerk;

Task::Task(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
           std::shared_ptr<Visualizer> visualizer, int n_controls)
    : tdef_loader_("hiqp_core", "hiqp::TaskDefinition"),
      tdyn_loader_("hiqp_core", "hiqp::TaskDynamics"),
      geom_prim_map_(geom_prim_map),
      visualizer_(visualizer),
      n_controls_(n_controls) {}

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

  def_params_ = def_params;
  dyn_params_ = dyn_params;

  def_->task_name_ = task_name_;
  def_->priority_ = priority_;
  def_->active_ = active_;
  def_->visible_ = visible_;

  dyn_->task_name_ = task_name_;
  dyn_->priority_ = priority_;
  dyn_->active_ = active_;
  dyn_->visible_ = visible_;

  // ROS_INFO("Definition created");
  if (def_->initialize(def_params, robot_state) != 0) {
    def_.reset();
    return -5;
  }
// ROS_INFO("Definition initalized");
  // ROS_INFO("Task Definition created.");

  if (dyn_->init(dyn_params, robot_state, def_->getInitialValue(),
                 def_->getFinalValue(robot_state)) != 0)
    return -6;
  // ROS_INFO("Task Dynamics created.");

  if (!checkConsistency(robot_state)) return -7;

  // The task was successfully setup
  // ROS_INFO("Done");
  return 0;
}

int Task::update(RobotStatePtr robot_state) {
  if (!checkConsistency(robot_state)) return -1;
  if (def_->update(robot_state) != 0) return -2;
  if (dyn_->update(robot_state, def_->e_, def_->J_) != 0) return -3;
  return 0;

  // DEBUG =============================================
  // std::cerr<<std::setprecision(2)<<"Update task
  // '"<<getTaskName()<<"'"<<std::endl;
  // std::cerr<<"J_t:"<<std::endl<<def_->J_<<std::endl;
  // std::cerr<<"signs: ";
  // for(int i=0; i<def_->task_types_.size();i++)
  //   std::cerr<<def_->task_types_[i]<<" ";

  // std::cerr<<std::endl<<"de*: "<<dyn_->e_dot_star_.transpose()<<std::endl;
  // DEBUG END ==========================================
}

int Task::constructDefinition(const std::vector<std::string>& def_params) {
  std::string type = def_params.at(0);

  if (type.compare("TDefFullPose") == 0) {
    def_ = std::make_shared<TDefFullPose>(geom_prim_map_, visualizer_);
  } else if (type.compare("TDefJntConfig") == 0) {
    def_ = std::make_shared<TDefJntConfig>(geom_prim_map_, visualizer_);
  } else if (type.compare("TDefJntLimits") == 0) {
    def_ = std::make_shared<TDefJntLimits>(geom_prim_map_, visualizer_);
  } else if (type.compare("TDefGeomProj") == 0) {
    std::string prim_type1 = def_params.at(1);
    std::string prim_type2 = def_params.at(2);
    if (prim_type1.compare("point") == 0 && prim_type2.compare("point") == 0) {
      def_ = std::make_shared<
          TDefGeometricProjection<GeometricPoint, GeometricPoint> >(
          geom_prim_map_, visualizer_);
    } else if (prim_type1.compare("point") == 0 &&
               prim_type2.compare("line") == 0) {
      def_ = std::make_shared<
          TDefGeometricProjection<GeometricPoint, GeometricLine> >(
          geom_prim_map_, visualizer_);
    } else if (prim_type1.compare("point") == 0 &&
               prim_type2.compare("plane") == 0) {
      def_ = std::make_shared<
          TDefGeometricProjection<GeometricPoint, GeometricPlane> >(
          geom_prim_map_, visualizer_);
    } else if (prim_type1.compare("point") == 0 &&
               prim_type2.compare("box") == 0) {
      def_ = std::make_shared<
          TDefGeometricProjection<GeometricPoint, GeometricBox> >(
          geom_prim_map_, visualizer_);
    } else if (prim_type1.compare("point") == 0 &&
               prim_type2.compare("cylinder") == 0) {
      def_ = std::make_shared<
          TDefGeometricProjection<GeometricPoint, GeometricCylinder> >(
          geom_prim_map_, visualizer_);
    } else if (prim_type1.compare("point") == 0 &&
               prim_type2.compare("sphere") == 0) {
      def_ = std::make_shared<
          TDefGeometricProjection<GeometricPoint, GeometricSphere> >(
          geom_prim_map_, visualizer_);
    } else if (prim_type1.compare("line") == 0 &&
               prim_type2.compare("line") == 0) {
      def_ = std::make_shared<
          TDefGeometricProjection<GeometricLine, GeometricLine> >(
          geom_prim_map_, visualizer_);
    } else if (prim_type1.compare("sphere") == 0 &&
               prim_type2.compare("plane") == 0) {
      def_ = std::make_shared<
          TDefGeometricProjection<GeometricSphere, GeometricPlane> >(
          geom_prim_map_, visualizer_);
    } else if (prim_type1.compare("sphere") == 0 &&
               prim_type2.compare("sphere") == 0) {
      def_ = std::make_shared<
          TDefGeometricProjection<GeometricSphere, GeometricSphere> >(
          geom_prim_map_, visualizer_);
    } else if (prim_type1.compare("frame") == 0 &&
               prim_type2.compare("frame") == 0) {
      def_ = std::make_shared<
          TDefGeometricProjection<GeometricFrame, GeometricFrame> >(
          geom_prim_map_, visualizer_);
    } else {
      printHiqpWarning(
          "TDefGeomProj does not support primitive combination of types '" +
          prim_type1 + "' and '" + prim_type2 + "'!");
      return -1;
    }
  } else if (type.compare("TDefGeomProjWithNullspace") == 0) {
    std::string prim_type1 = def_params.at(1);
    std::string prim_type2 = def_params.at(2);
    if (prim_type1.compare("point") == 0 && prim_type2.compare("cylinder") == 0) {
      def_ = std::make_shared<
          TDefGeometricProjectionWithNullspace<GeometricPoint, GeometricCylinder> >(
          geom_prim_map_, visualizer_);
    } else if (prim_type1.compare("point") == 0 && prim_type2.compare("plane") == 0) {
      def_ = std::make_shared<
          TDefGeometricProjectionWithNullspace<GeometricPoint, GeometricPlane> >(
          geom_prim_map_, visualizer_);
    } else {
      printHiqpWarning(
          "TDefGeometricProjectionWithNullspace does not support primitive combination of types '" +
          prim_type1 + "' and '" + prim_type2 + "'!");
      return -1;
    }
  } else if (type.compare("TDefGeomAlign") == 0) {
    std::string prim_type1 = def_params.at(1);
    std::string prim_type2 = def_params.at(2);
    if (prim_type1.compare("line") == 0 && prim_type2.compare("line") == 0) {
      def_ = std::make_shared<
          TDefGeometricAlignment<GeometricLine, GeometricLine> >(geom_prim_map_,
                                                                 visualizer_);
    } else if (prim_type1.compare("line") == 0 &&
               prim_type2.compare("plane") == 0) {
      def_ = std::make_shared<
          TDefGeometricAlignment<GeometricLine, GeometricPlane> >(
          geom_prim_map_, visualizer_);
    } else if (prim_type1.compare("line") == 0 &&
               prim_type2.compare("cylinder") == 0) {
      def_ = std::make_shared<
          TDefGeometricAlignment<GeometricLine, GeometricCylinder> >(
          geom_prim_map_, visualizer_);
    } else if (prim_type1.compare("line") == 0 &&
               prim_type2.compare("sphere") == 0) {
      def_ = std::make_shared<
          TDefGeometricAlignment<GeometricLine, GeometricSphere> >(
          geom_prim_map_, visualizer_);
    } else if (prim_type1.compare("frame") == 0 &&
               prim_type2.compare("frame") == 0) {
      def_ = std::make_shared<
          TDefGeometricAlignment<GeometricFrame, GeometricFrame> >(
          geom_prim_map_, visualizer_);
    } else {
      printHiqpWarning(
          "TDefGeomAlign does not support primitive combination of types '" +
          prim_type1 + "' and '" + prim_type2 + "'!");
      return -1;
    }
  } else if (type.compare("TDefMetaTask") == 0) {
    def_ = std::make_shared<TDefMetaTask>(geom_prim_map_, visualizer_);
  }
  else {
    try {
      def_ = std::shared_ptr<hiqp::TaskDefinition>(
          tdef_loader_.createClassInstance("hiqp::tasks::" + type));
      def_->initializeTaskDefinition(geom_prim_map_, visualizer_);
    } catch (pluginlib::PluginlibException& ex) {
      printHiqpWarning("The task definition type name '" + type +
                       "' was not understood!");
      ROS_ERROR("tdef_loader_ returned error: %s", ex.what());
      return -1;
    }
  }
  return 0;
}

int Task::constructDynamics(const std::vector<std::string>& dyn_params) {
  std::string type = dyn_params.at(0);

  if (type.compare("TDynLinear") == 0) {
    dyn_ = std::make_shared<TDynLinear>(geom_prim_map_, visualizer_);
  } else if (type.compare("TDynCubic") == 0) {
    dyn_ = std::make_shared<TDynCubic>(geom_prim_map_, visualizer_);
  } else if (type.compare("TDynJntLimits") == 0) {
    dyn_ = std::make_shared<TDynJntLimits>(geom_prim_map_, visualizer_);
  } else if (type.compare("TDynMinimalJerk") == 0) {
    dyn_ = std::make_shared<TDynMinimalJerk>(geom_prim_map_, visualizer_);
  } else if (type.compare("TDynHyperSin") == 0) {
    dyn_ = std::make_shared<TDynHyperSin>(geom_prim_map_, visualizer_);
  } else {
    try {
      dyn_ = std::shared_ptr<hiqp::TaskDynamics>(
          tdyn_loader_.createClassInstance("hiqp::tasks::" + type));
      dyn_->initializeTaskDynamics(geom_prim_map_, visualizer_);
    } catch (pluginlib::PluginlibException& ex) {
      ROS_ERROR("The plugin failed to load for some reason. Error: %s",
                ex.what());
      printHiqpWarning("The task dynamics type name '" + type +
                       "' was not understood!");
      return -1;
    }
  }

  return 0;
}

bool Task::checkConsistency(RobotStatePtr robot_state) {
  if (!def_) {
    printHiqpWarning("The task '" + task_name_ +
                     "' is inconsistent after initialization. The task "
                     "definition was not constructed properly.");
    return false;
  }

  if (!dyn_) {
    printHiqpWarning("The task '" + task_name_ +
                     "' is inconsistent after initialization. The task "
                     "dynamics was not constructed properly.");
    return false;
  }

  if (def_->e_.size() != def_->J_.rows()) {
    printHiqpWarning(
        "The task '" + task_name_ +
        "' is inconsistent after initialization (dimension mismatch). " +
        "Size of task function (e_.size()) is " +
        std::to_string(def_->e_.size()) + ", " +
        "number of rows of task jacobian (J_.rows()) is " +
        std::to_string(def_->J_.rows()));
    return false;
  }

  if (def_->task_types_.size() != def_->J_.rows()) {
    ROS_WARN_THROTTLE(
        3,
        "The task '%s' is inconsistent after initialization (dimension "
        "mismatch). Size of task types array (task_types_.size()) is %ld, "
        "number of rows of task jacobian (J_.rows()) is %ld.",
        task_name_.c_str(), def_->task_types_.size(), def_->J_.rows());
    return false;
  }

  if (dyn_->e_dot_star_.size() != def_->J_.rows()) {
    printHiqpWarning(
        "The task '" + task_name_ +
        "' is inconsistent after initialization (dimension mismatch). " +
        "Size of desired task dynamics (e_dot_star_.size()) is " +
        std::to_string(dyn_->e_dot_star_.size()) + ", " +
        "number of rows of task jacobian (J_.rows()) is " +
        std::to_string(def_->J_.rows()));
    return false;
  }

  if (def_->J_.cols() != robot_state->getNumJoints()) {
    printHiqpWarning(
        "The task '" + task_name_ +
        "' is inconsistent after initialization (dimension mismatch). " +
        "Number of columns of task jacobian (J_.cols()) is " +
        std::to_string(def_->J_.cols()) + ", " +
        "total number of joints in the robot is " +
        std::to_string(robot_state->getNumJoints()));
    return false;
  }

  // the task definition and dynamics are consistent
  return true;
}

}  // namespace hiqp
