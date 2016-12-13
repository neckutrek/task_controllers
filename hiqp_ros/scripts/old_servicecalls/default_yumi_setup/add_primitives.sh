rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'mypoint1'
type: 'point'
frame_id: 'gripper_r_base'
visible: true
color: [0.2, 0.2, 1.0, 0.9]
parameters: [0.0, 0.0, 0.1]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'mypoint2'
type: 'point'
frame_id: 'world'
visible: true
color: [0.2, 0.2, 1.0, 0.9]
parameters: [0.3, 0.0, 0.0]"