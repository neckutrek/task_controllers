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

#ifndef HIQP_TASK_MANAGER_H
#define HIQP_TASK_MANAGER_H

#include <vector>
#include <memory>

#include <hiqp/task.h>
#include <hiqp/visualizer.h>
#include <hiqp/hiqp_solver.h>
#include <hiqp/robot_state.h>
#include <hiqp/geometric_primitives/geometric_primitive_map.h>
#include <hiqp/solvers/casadi_solver.h>
#include <hiqp/solvers/gurobi_solver.h>
#include <kdl/tree.hpp>
#include <kdl/jntarrayvel.hpp>

namespace hiqp {

  /*! \brief A structure used to get data out of tasks that is to be monitored.
   *  \author Marcus A Johansson */
  class TaskMeasure {
  public:
    TaskMeasure(const std::string& task_name,
                       const Eigen::VectorXd& e,
                       const Eigen::VectorXd& de,
                       const Eigen::VectorXd& pm)
    : task_name_(task_name), e_(e), de_(de), pm_(pm) {}

    std::string         task_name_;
    Eigen::VectorXd     e_;
    Eigen::VectorXd     de_;
    Eigen::VectorXd     pm_;
  };

  /*! \brief The central mediator class in the HiQP framework.
   *  \author Marcus A Johansson */  
  class TaskManager {
  public:
    TaskManager(std::shared_ptr<Visualizer> visualizer);
    ~TaskManager() noexcept;

    void init(unsigned int n_controls);

    /*! \brief Generates controls from a particular robot state. */
    bool getVelocityControls(RobotStatePtr robot_state,
                             std::vector<double> &controls);

    /*! \brief Retrieves the performance measures for every active task along
     *         with the task's name and unique identifier. */
    void getTaskMeasures(std::vector<TaskMeasure>& data);

    /*! \brief Adds a new task to the task manager or updates an existing one
     *  \return 0 if the task creation was successful,
     *          -1 is the behaviour_parameters argument was invalid,
     *          -2 if the task behaviour name was not recognised,
     *          -3 if the task name was not recognised */
    int setTask(const std::string& task_name,
                unsigned int priority,
                bool visible,
                bool active,
                const std::vector<std::string>& def_params,
                const std::vector<std::string>& dyn_params,
                RobotStatePtr robot_state);

    int removeTask(std::string task_name);
    int removeAllTasks();
    int listAllTasks();

    inline void activateTask(const std::string& name) { task_map_.find(name)->second->setActive(true); }
    inline void deactivateTask(const std::string& name) { task_map_.find(name)->second->setActive(false); }

    void renderPrimitives();

    int addGeometricPrimitive(const std::string& name,
                              const std::string& type,
                              const std::string& frame_id,
                              bool visible,
                              const std::vector<double>& color,
                              const std::vector<double>& parameters);

    int removeGeometricPrimitive(std::string name);

    int removeAllGeometricPrimitives();

    int listAllGeometricPrimitives();

    inline std::shared_ptr<GeometricPrimitiveMap> getGeometricPrimitiveMap() { return geometric_primitive_map_; }


   private:
    typedef std::map< std::string, std::shared_ptr<Task> > TaskMap;

    std::shared_ptr<GeometricPrimitiveMap>       geometric_primitive_map_;
    std::shared_ptr<Visualizer>                  visualizer_;

    TaskMap                                      task_map_;

    std::shared_ptr<HiQPSolver>                  solver_;

    unsigned int                                 n_controls_;

    TaskManager(const TaskManager& other) = delete;
    TaskManager(TaskManager&& other) = delete;
    TaskManager& operator=(const TaskManager& other) = delete;
    TaskManager& operator=(TaskManager&& other) noexcept = delete;
  };

} // namespace hiqp

#endif
