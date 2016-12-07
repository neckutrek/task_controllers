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

#ifndef TASK_H
#define TASK_H

#include <string>
#include <vector>
#include <memory>

#include <hiqp/geometric_primitives/geometric_primitive_map.h>
#include <hiqp/visualizer.h>
#include <hiqp/task_definition.h>
#include <hiqp/task_dynamics.h>

#include <Eigen/Dense>

namespace hiqp {

  /*! \brief A Task has a TaskDefinition and a TaskDynamics.
   *  \author Marcus A Johansson */
  class Task {
  public:
    Task(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
         std::shared_ptr<Visualizer> visualizer,
         int n_controls);

    ~Task() noexcept {}

    /*! \brief This should be called after all fields of Task are set properly. */
    int init(const std::vector<std::string>& def_params,
             const std::vector<std::string>& dyn_params,
             RobotStatePtr robot_state);

    inline void         setTaskName(const std::string& name) { task_name_ = name; }
    inline std::string  getTaskName()                      { return task_name_; }
    inline void         setPriority(unsigned int priority) { priority_ = priority; }
    inline unsigned int getPriority()                      { return priority_; }
    inline void         setActive(bool active)             { active_ = active; }
    inline bool         getActive()                        { return active_; }
    inline void         setVisible(bool visible)           { visible_ = visible; }
    inline bool         getVisible()                       { return visible_; }
    inline unsigned int getDimensions()                    { if (def_) return def_->getDimensions(); else return 0; }

    /*! \brief Recomputes the task performance value, jacobian and its dynamics. */
    void update(RobotStatePtr robot_state);

    void monitor() {if (def_) def_->monitor(); if (dyn_) dyn_->monitor();}

    /*! \brief Returns the task function performance values as a vector. */
    /// \todo should be renamed to getTaskFunctionValue
    Eigen::VectorXd getValue() const      
      { if (def_) return def_->e_; else return Eigen::VectorXd(); }

    /*! \brief Returns the task jacobian as a matrix. */
    Eigen::MatrixXd getJacobian() const   
      { if (def_) return def_->J_; else return Eigen::MatrixXd(); }

    /*! \brief Returns the task dynamics as a vector. */
    Eigen::VectorXd getDynamics() const   
      { if (dyn_) return dyn_->e_dot_star_; else return Eigen::VectorXd(); }

    /*! \brief Returns the task types (leq/eq/geq task) for each dimension of the task space. Returns a vector or -1, 0 or 1 for leq, eq and geq tasks respectively. */
    /// \todo Change from std::vector<int> to Eigen::VectorXd - should also be renamed to getTaskSenses()
    std::vector<int> getTaskTypes() const  
      { if (def_) return def_->task_types_; else return std::vector<int>(); }

    /*! \brief Returns the user-defined custom performance measures defined in the monitor() member function in a child class of TaskDefinition. */
    Eigen::VectorXd getPerformanceMeasures() const 
      { if (def_) return def_->performance_measures_; else return Eigen::VectorXd(); }

  private:
    Task(const Task& other) = delete;
    Task(Task&& other) = delete;
    Task& operator=(const Task& other) = delete;
    Task& operator=(Task&& other) noexcept = delete;

    int constructDefinition(const std::vector<std::string>& def_params);
    int constructDynamics(const std::vector<std::string>& dyn_params);

    std::shared_ptr<TaskDefinition>          def_;
    std::shared_ptr<TaskDynamics>            dyn_;

    std::shared_ptr<GeometricPrimitiveMap>   geom_prim_map_;
    std::shared_ptr<Visualizer>              visualizer_;

    unsigned int                             n_controls_;
    std::string                              task_name_;
    unsigned int                             priority_;
    bool                                     active_;
    bool                                     visible_;
  };

} // namespace hiqp

#endif // include guard
