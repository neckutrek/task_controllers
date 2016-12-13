rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'set_starting_pos'
type: 'TaskFullPose'
behaviour: ['DynamicsFirstOrder', '1.0']
priority: 3
visibility: 0
active: 1
parameters: ['-0.42', '-1.48', '1.21', '0.75', '-0.8', '0.45', '1.21', '0', '0', '0.42', '-1.48', '-1.21', '0.75', '0.8', '0.45', '1.21', '0', '0']"

sleep 5.0

#rosservice call /yumi/hiqp_kinematics_controller/remove_task "task_name: set_starting_pos"