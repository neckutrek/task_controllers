rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'mypoint1'
type: 'point'
frame_id: 'gripper_r_base'
visible: true
color: [0.0, 1.0, 0.0, 0.9]
parameters: [0, 0, 0.2]"


rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'mypoint2'
type: 'point'
frame_id: 'gripper_l_base'
visible: true
color: [0.0, 1.0, 0.0, 0.9]
parameters: [0.0, 0, 0.2]"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'minjerk_task'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsMinimalJerk', '10.0', '0.1']
priority: 3
visibility: 0
parameters: ['point', 'point', 'mypoint1 = mypoint2']"