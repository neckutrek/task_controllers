# The HiQP Control Framework
Copyright (C) 2016-2017 Marcus A. Johansson

HiQP is a whole-body control framework for computing inverse kinematics (and dynamics in future releases) involving multiple objectives which can be incompatible. It uses the inequality task function approach in [1] and is based on hierarchical least-squares optimization. Lower-ranked tasks are fulfilled as good as possible (in the least-square sense) in the null-space of higher-ranked tasks. More details are given in [2] - available at http://www.diva-portal.org/smash/get/diva2:1056500/FULLTEXT01.pdf. 

## Acknowledgements
Robert Krug and Todor Stoyanov at the AASS Research Institute at Örebro University, Sweden, have been important contributors to this project. Thank you.

## Upcoming features/changes
- A Wiki/user guide will be created as a tutorial on how to use HiQP with ROS. In the meantime, we refer to the hiqp_demos package for examples of how to use HiQP: git@github.com:OrebroUniversity/hiqp_demos.git
- The list of available service calls will be revised and extended.
- A capsule primitive will be added to the set of geometric primitives.
- The framework will be extended to compute controls on a joint acceleration level in order to enable inverse dynamics control. 

## Installation note

HiQP relies on qpOASES to solve Quadratic Programs (QPs). qpOASES comes under the GNU LGPL v3 license. 

## How to cite HiQP

A publication presenting HiQP is in preparation. In the meantime, please refer to [2].

</br>
[1] ... O. Kanoun, F. Lamiraux and P.-B. Wieber, Kinematic control of redundant manipulators: Generalizing the task-priority framework to inequality task. IEEE T-RO, 27(4):785-792, 2011.
</br>
[2] ... M. A. Johansson, Online whole-body control using hierarchical quadratic programming: implementation and evaluation of the HiQP control framework. MSc Thesis, 2016.
