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

#ifndef HIQP_TASK_DEFINITION_H
#define HIQP_TASK_DEFINITION_H

#include <iostream>
#include <vector>
#include <memory>

#include <hiqp/geometric_primitives/geometric_primitive_map.h>
#include <hiqp/visualizer.h>
#include <hiqp/robot_state.h>

#include <Eigen/Dense>

namespace hiqp
{

  using geometric_primitives::GeometricPrimitiveMap;

  class Task;

  /*! \brief Defines the task space and the performance value of a task.
   *  \author Marcus A Johansson */ 
  class TaskDefinition
  {
  public:
    TaskDefinition(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
                   std::shared_ptr<Visualizer> visualizer) 
    : geometric_primitive_map_(geom_prim_map), visualizer_(visualizer) {}

    ~TaskDefinition() noexcept {}

    virtual int init(const std::vector<std::string>& parameters,
                     RobotStatePtr robot_state,
                     unsigned int n_controls) = 0;

    virtual int update(RobotStatePtr robot_state) = 0;

    virtual int monitor() = 0;

    unsigned int            getDimensions()       { return n_dimensions_; }
    Eigen::VectorXd         getInitialValue()     { return e_initial_; }
    Eigen::MatrixXd         getInitialJacobian()  { return J_initial_; }
    /// \todo rename to getFinalTaskFunctionValue
    virtual Eigen::VectorXd getFinalValue(RobotStatePtr robot_state)
      { return Eigen::VectorXd::Zero(e_.rows()); }

  protected:
    Eigen::VectorXd                 e_; // the performance value of the task
    Eigen::MatrixXd                 J_; // the task jacobian
    /// \todo rename to task_senses_
    std::vector<int>                task_types_; // -1 leq, 0 eq, 1 geq
    Eigen::VectorXd                 performance_measures_;
    unsigned int                    n_dimensions_;

    inline std::string  getTaskName()                      { return task_name_; }
    inline unsigned int getPriority()                      { return priority_; }
    inline bool         getActive()                        { return active_; }
    inline bool         getVisible()                       { return visible_; }
    inline std::shared_ptr<Visualizer> 
                        getVisualizer()                    { return visualizer_; }
    inline std::shared_ptr<GeometricPrimitiveMap> 
                        getGeometricPrimitiveMap()         { return geometric_primitive_map_; }

  private:
    friend                                    Task;

    std::shared_ptr<GeometricPrimitiveMap>    geometric_primitive_map_;
    std::shared_ptr<Visualizer>               visualizer_;

    Eigen::VectorXd                           e_initial_;
    Eigen::MatrixXd                           J_initial_;

    std::string                               task_name_;
    unsigned int                              priority_;
    bool                                      visible_;
    bool                                      active_;

    TaskDefinition(const TaskDefinition& other) = delete;
    TaskDefinition(TaskDefinition&& other) = delete;
    TaskDefinition& operator=(const TaskDefinition& other) = delete;
    TaskDefinition& operator=(TaskDefinition&& other) noexcept = delete;

    /*! \brief Calls init() of the child class, and properly sets up the TaskDefinition object. */
    int initialize(const std::vector<std::string>& parameters,
                   RobotStatePtr robot_state,
                   unsigned int n_controls) {
        if (init(parameters, robot_state, n_controls) != 0)
          return -1;
        update(robot_state);
        e_initial_ = e_;
        J_initial_ = J_;
        return 0;
    }
  };

} // namespace hiqp

#endif // include guard
