#!/bin/sh

counter=0
for spy in -0.45 -0.225 0 0.225 0.45
do
  for spz in 0.25 0.425 0.6 0.775 0.95
  do
    counter=$(($counter + 1))
    rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'experiment_starting_point_$counter'
type: 'sphere'
frame_id: 'world'
visible: true
color: [0.0, 0.0, 0.0, 1.0]
parameters: [0.2, $spy, $spz, 0.01]"
  done
done

counter=0
for cpx in 0.077 0.191 0.305 0.419 0.533
do
  for cpy in -0.533 -0.3415 -0.15 0.0415 0.233
  do
    counter=$(($counter + 1))
    rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'experiment_cylinder_$counter'
type: 'cylinder'
frame_id: 'world'
visible: true
color: [0.0, 1.0, 0.0, 0.6]
parameters: [0.0, 0.0, 1.0, $cpx, $cpy, 0.115, 0.033, 0.1]"
  done
done