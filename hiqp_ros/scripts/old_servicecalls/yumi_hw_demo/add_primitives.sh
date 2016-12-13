rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'yumi_r_gripper_ee_point'
type: 'point'
frame_id: 'gripper_r_base'
visible: true
color: [0.0, 1.0, 0.0, 0.9]
parameters: [0.0, 0, 0.1]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'yumi_r_gripper_ee_line'
type: 'line'
frame_id: 'gripper_r_base'
visible: true
color: [0.0, 1.0, 0.0, 0.9]
parameters: [0, 0, 1, 0, 0, 0.1]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'thecylinder'
type: 'cylinder'
frame_id: 'world'
visible: true
color: [0.0, 1.0, 0.0, 0.5]
parameters: [0, 0, 1, 1.0, 0, 0, 0.1, 0.3]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'topplane'
type: 'plane'
frame_id: 'world'
visible: true
color: [0.0, 0.0, 1.0, 0.3]
parameters: [0, 0, 1, 0.3]"


