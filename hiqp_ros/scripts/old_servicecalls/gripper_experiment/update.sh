rosservice call /yumi/hiqp_kinematics_controller/update_task \
"name: 'bring_back_to_start'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsFirstOrder', '5']
priority: 2
visibility: 0
active: 1
parameters: ['point', 'point', 'experiment_gripper_point = experiment_starting_point']"