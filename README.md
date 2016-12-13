# The HiQP Control Framework
Copyright (C) 2016 Marcus A Johansson

HiQP is an optimal control framework targeted at robotics. It is based on the task function approach.

## Acknowledgements
Robert Krug and Todor Stoyanov at the AASS Researche Institute at Örebro University, Sweden, have been important contributors to this project. Thank you.

## Upcoming features/changes
- A Wiki will be created as a tutorial on how to use HiQP with ROS.
- The list of available service calls will be revised and extended.
- A capsule primitive will be added to the set of geometric primitives.
- An internal representation of the dynamics model of the robot will be added to allow for effort control calculations.
- A joint effort control interface based ROS-controller will be added.
- A prediction horisont will be added to extend the controller to an MPC. This will enable motion optimization over multiple time steps and also allow for predetermination of future unsatisfied constraints.

## Known issues
- Geometric projection for point on box is not working when the box is rotated. The projected point is not positioned on the box.

## How to suppress gurobi printouts from casadi
In ``casadi/casadi/interfaces/gurobi/gurobi_interface.cpp`` at line 116, add ``GRBsetintparam(m->env, "OutputFlag", 0);``