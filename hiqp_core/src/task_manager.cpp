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

#include <hiqp/geometric_primitives/geometric_primitive_couter.h>
#include <hiqp/geometric_primitives/geometric_primitive_visualizer.h>
#include <hiqp/task_manager.h>
#include <hiqp/utilities.h>
#include <ros/console.h>
#include <iomanip>  // std::setw

#ifdef HIQP_CASADI
#include <hiqp/solvers/casadi_solver.h>
#endif
#ifdef HIQP_GUROBI
#include <hiqp/solvers/gurobi_solver.h>
#endif

#include <Eigen/Dense>

using hiqp::geometric_primitives::GeometricPrimitiveVisualizer;
using hiqp::geometric_primitives::GeometricPrimitiveCouter;

namespace hiqp {

TaskManager::TaskManager(std::shared_ptr<Visualizer> visualizer)
    : visualizer_(visualizer) {
  geometric_primitive_map_ = std::make_shared<GeometricPrimitiveMap>();
#ifdef HIQP_CASADI
  solver_ = std::make_shared<CasadiSolver>();
#endif
#ifdef HIQP_GUROBI
  solver_ = std::make_shared<GurobiSolver>();
#endif
}

TaskManager::~TaskManager() noexcept {}

void TaskManager::init(unsigned int n_controls) { n_controls_ = n_controls; }

bool TaskManager::getVelocityControls(RobotStatePtr robot_state,
                                      std::vector<double>& controls) {
  if (task_map_.size() < 1) {
    for (int i = 0; i < controls.size(); ++i) controls.at(i) = 0;

    return false;
  }

  solver_->clearStages();

  resource_mutex_.lock();
  for (auto&& kv : task_map_) {
    if (kv.second->getActive()) {
      if (kv.second->update(robot_state) == 0) {
        solver_->appendStage(kv.second->getPriority(), kv.second->getDynamics(),
                             kv.second->getJacobian(),
                             kv.second->getTaskTypes());
      }
    }
  }
  resource_mutex_.unlock();

  if (!solver_->solve(controls)) {
    ROS_WARN_THROTTLE(
        10,
        "Unable to solve the hierarchical QP, setting the velocity controls to "
        "zero!");
    for (int i = 0; i < controls.size(); ++i) controls.at(i) = 0;

    return false;
  }

  return true;
}

void TaskManager::getTaskMeasures(std::vector<TaskMeasure>& data) {
  data.clear();
  resource_mutex_.lock();
  for (auto&& kv : task_map_) {
    if (kv.second->getMonitored()) {
      kv.second->monitor();
      data.push_back(TaskMeasure(
          kv.second->getTaskName(), kv.second->getTaskTypes()[0],
          kv.second->getValue(), kv.second->getDynamics(),
          kv.second->getPerformanceMeasures()));
    }
  }
  resource_mutex_.unlock();
}

void TaskManager::renderPrimitives() {
  resource_mutex_.lock();
  GeometricPrimitiveVisualizer geom_prim_vis(visualizer_, 0);
  geometric_primitive_map_->acceptVisitor(geom_prim_vis);
  resource_mutex_.unlock();
}

int TaskManager::setTask(const std::string& task_name, unsigned int priority,
                         bool visible, bool active, bool monitored,
                         const std::vector<std::string>& def_params,
                         const std::vector<std::string>& dyn_params,
                         RobotStatePtr robot_state) {
  resource_mutex_.lock();
  std::shared_ptr<Task> task;
  std::string action = "Added";

  TaskMap::iterator it = task_map_.find(task_name);
  if (it == task_map_.end()) {
    task = std::make_shared<Task>(geometric_primitive_map_, visualizer_,
                                  n_controls_);
  } else {
    task = it->second;
    action = "Updated";
  }

  task->setTaskName(task_name);
  task->setPriority(priority);
  task->setVisible(visible);
  task->setActive(active);
  task->setMonitored(monitored);
  if (task->init(def_params, dyn_params, robot_state) != 0) {
    // printHiqpWarning("The task '" + task_name + "' was not added!");
    resource_mutex_.unlock();
    return -1;
  } else {
    task_map_.emplace(task_name, task);
    printHiqpInfo(action + " task '" + task_name + "'");
  }
  resource_mutex_.unlock();
  return 0;
}

int TaskManager::removeTask(std::string task_name) {
  resource_mutex_.lock();
  if (task_map_.erase(task_name) == 1) {
    geometric_primitive_map_->removeDependency(task_name);
    resource_mutex_.unlock();
    return 0;
  }
  resource_mutex_.unlock();
  return -1;
}

int TaskManager::removeAllTasks() {
  resource_mutex_.lock();
  TaskMap::iterator it = task_map_.begin();
  while (it != task_map_.end()) {
    geometric_primitive_map_->removeDependency(it->first);
    ++it;
  }
  task_map_.clear();
  resource_mutex_.unlock();
  return 0;
}

std::vector<TaskInfo> TaskManager::getAllTaskInfo() {
  resource_mutex_.lock();
  std::vector<TaskInfo> all_task_info;
  for (auto it : task_map_) {
    all_task_info.push_back(
        TaskInfo(it.second->getTaskName(), it.second->getPriority(),
                 it.second->getActive(), it.second->getMonitored(),
                 it.second->getDefParams(), it.second->getDynParams()));
  }

  resource_mutex_.unlock();
  return all_task_info;
}

int TaskManager::listAllTasks() {
  resource_mutex_.lock();
  int longest_name_length = 0;
  TaskMap::iterator it = task_map_.begin();
  while (it != task_map_.end()) {
    if (it->first.size() > longest_name_length)
      longest_name_length = it->first.size();
    ++it;
  }

  std::multimap<unsigned int, std::string> task_info_map;
  it = task_map_.begin();
  while (it != task_map_.end()) {
    std::stringstream ss;
    ss << std::setw(8) << it->second->getPriority() << " | "
       << std::setw(longest_name_length) << it->first << " | " << std::setw(6)
       << it->second->getActive() << " | " << std::setw(9)
       << it->second->getMonitored();
    task_info_map.emplace(it->second->getPriority(), ss.str());
    ++it;
  }
  resource_mutex_.unlock();

  std::cout << " - - - LISTING ALL REGISTERED TASKS - - -\n";
  std::cout << "Priority | Unique name";
  for (int i = 0; i < longest_name_length - 11; ++i) std::cout << " ";
  std::cout << " | Active | Monitored\n";
  std::cout << "----------------------";
  for (int i = 0; i < longest_name_length - 11; ++i) std::cout << "-";
  std::cout << "---------------------\n";

  for (auto&& info : task_info_map) {
    std::cout << info.second << "\n";
  }

  return 0;
}

int TaskManager::activateTask(const std::string& task_name) {
  resource_mutex_.lock();
  TaskMap::iterator it = task_map_.find(task_name);
  if (it != task_map_.end()) {
    it->second->setActive(true);
  } else {
    printHiqpWarning("When trying to activate task '" + task_name +
                     "': No task with that name found.");
  }
  resource_mutex_.unlock();
}

int TaskManager::deactivateTask(const std::string& task_name) {
  resource_mutex_.lock();
  TaskMap::iterator it = task_map_.find(task_name);
  if (it != task_map_.end()) {
    it->second->setActive(false);
  } else {
    printHiqpWarning("When trying to deactivate task '" + task_name +
                     "': No task with that name found.");
  }
  resource_mutex_.unlock();
}

int TaskManager::monitorTask(const std::string& task_name) {
  resource_mutex_.lock();
  TaskMap::iterator it = task_map_.find(task_name);
  if (it != task_map_.end()) {
    it->second->setMonitored(true);
  } else {
    printHiqpWarning("When trying to activate monitoring of task '" +
                     task_name + "': No task with that name found.");
  }
  resource_mutex_.unlock();
}

int TaskManager::demonitorTask(const std::string& task_name) {
  resource_mutex_.lock();
  TaskMap::iterator it = task_map_.find(task_name);
  if (it != task_map_.end()) {
    it->second->setMonitored(false);
  } else {
    printHiqpWarning("When trying to deactivate monitoring of task '" +
                     task_name + "': No task with that name found.");
  }
  resource_mutex_.unlock();
}

int TaskManager::setPrimitive(const std::string& name, const std::string& type,
                              const std::string& frame_id, bool visible,
                              const std::vector<double>& color,
                              const std::vector<double>& parameters) {
  resource_mutex_.lock();
  geometric_primitive_map_->setGeometricPrimitive(name, type, frame_id, visible,
                                                  color, parameters);
  resource_mutex_.unlock();
  return 0;
}

int TaskManager::removePrimitive(std::string name) {
  resource_mutex_.lock();
  GeometricPrimitiveVisualizer geom_prim_vis(visualizer_, 1);
  geometric_primitive_map_->acceptVisitor(geom_prim_vis, name);
  geom_prim_vis.removeAllVisitedPrimitives();
  geometric_primitive_map_->removeGeometricPrimitive(name);
  resource_mutex_.unlock();
  return 0;
}

int TaskManager::removeAllPrimitives() {
  resource_mutex_.lock();
  GeometricPrimitiveVisualizer geom_prim_vis(visualizer_, 1);
  geometric_primitive_map_->acceptVisitor(geom_prim_vis);
  geom_prim_vis.removeAllVisitedPrimitives();
  geometric_primitive_map_->clear();
  resource_mutex_.unlock();
  return 0;
}

int TaskManager::listAllPrimitives() {
  resource_mutex_.lock();
  std::cout << "LISTING ALL REGISTERED GEOMETRIC PRIMITIVES:\n";
  std::cout << "Name | Frame ID | Visible | Visual ID | Type\n";
  GeometricPrimitiveCouter geom_prim_cout;
  geometric_primitive_map_->acceptVisitor(geom_prim_cout);
  resource_mutex_.unlock();
  return 0;
}

std::vector<PrimitiveInfo> TaskManager::getAllPrimitiveInfo() {
  resource_mutex_.lock();
  std::vector<PrimitiveInfo> all_primitive_info;
  all_primitive_info = this->geometric_primitive_map_->getAllPrimitiveInfo();
  resource_mutex_.unlock();
  return all_primitive_info;
}

int TaskManager::removePriorityLevel(unsigned int priority) {
  resource_mutex_.lock();
  TaskMap::iterator it = task_map_.begin();
  while (it != task_map_.end()) {
    if (it->second->getPriority() == priority) {
      geometric_primitive_map_->removeDependency(it->second->getTaskName());
      it = task_map_.erase(it);
    } else {
      ++it;
    }
  }
  resource_mutex_.unlock();
}

int TaskManager::activatePriorityLevel(unsigned int priority) {
  resource_mutex_.lock();
  for (auto&& kv : task_map_) {
    if (kv.second->getPriority() == priority) kv.second->setActive(true);
  }
  resource_mutex_.unlock();
}

int TaskManager::deactivatePriorityLevel(unsigned int priority) {
  resource_mutex_.lock();
  for (auto&& kv : task_map_) {
    if (kv.second->getPriority() == priority) kv.second->setActive(false);
  }
  resource_mutex_.unlock();
}

int TaskManager::monitorPriorityLevel(unsigned int priority) {
  resource_mutex_.lock();
  for (auto&& kv : task_map_) {
    if (kv.second->getPriority() == priority) kv.second->setMonitored(true);
  }
  resource_mutex_.unlock();
}

int TaskManager::demonitorPriorityLevel(unsigned int priority) {
  resource_mutex_.lock();
  for (auto&& kv : task_map_) {
    if (kv.second->getPriority() == priority) kv.second->setMonitored(false);
  }
  resource_mutex_.unlock();
}

}  // namespace hiqp
