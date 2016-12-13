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

#include <iomanip> // std::setw
#include <ros/console.h>
#include <hiqp/task_manager.h>
#include <hiqp/utilities.h>
//#include <hiqp/tasks/task_geometric_projection.h>
//#include <hiqp/tasks/dynamics_first_order.h>

#include <Eigen/Dense>

namespace hiqp {

TaskManager::TaskManager(std::shared_ptr<Visualizer> visualizer)
: visualizer_(visualizer) {
  geometric_primitive_map_ = std::make_shared<GeometricPrimitiveMap>();
  solver_ = std::make_shared<GurobiSolver>();
}

TaskManager::~TaskManager() noexcept {}

void TaskManager::init(unsigned int n_controls) {
  n_controls_ = n_controls; 
}

bool TaskManager::getVelocityControls(RobotStatePtr robot_state,
                                      std::vector<double> &controls) {
  if (task_map_.size() < 1) {
    for (int i=0; i<controls.size(); ++i)
      controls.at(i) = 0;
    
    return false;
  }

  solver_->clearStages();

  for (auto&& kv : task_map_) {
    if (kv.second->getActive()) {
      if(kv.second->update(robot_state) !=0){
	printHiqpWarning("Task '" + kv.second->getTaskName() + "' could not be updated and will be ignored while solving for controls!");
	continue;
      }

      solver_->appendStage(kv.second->getPriority(), 
                           kv.second->getDynamics(), 
                           kv.second->getJacobian(),
                           kv.second->getTaskTypes());
    }
  }
  if (!solver_->solve(controls)) {
    printHiqpWarning("Unable to solve the hierarchical QP, setting the velocity controls to zero!");
    for (int i=0; i<controls.size(); ++i)
      controls.at(i) = 0;

    return false;
  }

  return true;
}

void TaskManager::getTaskMeasures(std::vector<TaskMeasure>& data) {
  data.clear();
  for (auto&& kv : task_map_) {
    kv.second->monitor();
    data.push_back(TaskMeasure(kv.second->getTaskName(), 
                               kv.second->getValue(),
                               kv.second->getDynamics(),
                               kv.second->getPerformanceMeasures()));
  }
}

int TaskManager::setTask(const std::string& task_name,
                         unsigned int priority,
                         bool visible,
                         bool active,
                         const std::vector<std::string>& def_params,
                         const std::vector<std::string>& dyn_params,
                         RobotStatePtr robot_state) {  
  std::shared_ptr<Task> task;
  std::string action = "Added";

  TaskMap::iterator it = task_map_.find(task_name);
  if (it == task_map_.end()) {
    task = std::make_shared<Task>(geometric_primitive_map_, visualizer_, n_controls_);
  } else {
    task = it->second;
    action = "Updated";
  }

  task->setTaskName(task_name);
  task->setPriority(priority);
  task->setVisible(visible);
  task->setActive(active);

  if (task->init(def_params, dyn_params, robot_state) != 0) {
    //printHiqpWarning("The task '" + task_name + "' was not added!");
    return -1;
  } else {
    task_map_.emplace(task_name, task);
    printHiqpInfo(action + " task '" + task_name + "'");
  }
  return 0;
}

int TaskManager::removeTask(std::string task_name) {
  if (task_map_.erase(task_name) == 1) 
  {
    geometric_primitive_map_->removeDependency(task_name);
    return 0;
  }

  return -1;
}

int TaskManager::removeAllTasks() {
  TaskMap::iterator it = task_map_.begin();
  while (it != task_map_.end()) {
    geometric_primitive_map_->removeDependency(it->first);
    ++it;
  }
  task_map_.clear();
  return 0;
}

int TaskManager::listAllTasks() {
  std::cout << "LISTING ALL REGISTERED TASKS:\n";
  TaskMap::iterator it = task_map_.begin();

  int longest_name_length = 0;
  while (it != task_map_.end()) {
    if (it->first.size() > longest_name_length)
      longest_name_length = it->first.size();
    ++it;
  }

  std::cout << "Priority | Unique name"; 
  for (int i=0; i<longest_name_length-11; ++i)
    std::cout << " ";
  std::cout << " | Active\n";
  std::cout << "-------------------------------\n";

  it = task_map_.begin();
  while (it != task_map_.end()) {
    std::cout 
      << std::setw(8) << it->second->getPriority() << " | "
      << std::setw(longest_name_length) << it->first << " | "
      << std::setw(6) << it->second->getActive() << "\n";
    ++it;
  }
  return 0;
}





int TaskManager::addGeometricPrimitive
(
  const std::string& name,
  const std::string& type,
  const std::string& frame_id,
  bool visible,
  const std::vector<double>& color,
  const std::vector<double>& parameters
)
{

  geometric_primitive_map_->addGeometricPrimitive(
    name, type, frame_id, visible, color, parameters);

  return 0;
}





int TaskManager::removeGeometricPrimitive
(
  std::string name
)
{
  return geometric_primitive_map_->removeGeometricPrimitive(name);
}




int TaskManager::removeAllGeometricPrimitives
()
{
  geometric_primitive_map_->clear();
}





} // namespace hiqp


