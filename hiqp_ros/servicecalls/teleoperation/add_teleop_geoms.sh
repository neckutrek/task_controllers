rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'teleop_wintracker_frame'
type: 'frame'
frame_id: 'yumi_body'
visible: true
color: [0, 0, 0, 1]
parameters: [0, 0, 0, 1, 0, 0, 0]"

rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'teleop_gripper_frame'
type: 'frame'
frame_id: 'gripper_r_base'
visible: true
color: [0, 0, 0, 1]
parameters: [0, 0, 0, 1, 0, 0, 0]"

rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'object_vertical_axis'
type: 'line'
frame_id: 'yumi_body'
visible: true
color: [1.0, 0.0, 0.0, 0.9]
parameters: [0, 0, 1, 0.45, 0, 0.1]"

rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'gripper_approach_axis'
type: 'line'
frame_id: 'gripper_r_base'
visible: true
color: [0.0, 1.0, 0.0, 0.9]
parameters: [0, 0, 1, 0, 0, 0]"

rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'gripper_vertical_axis'
type: 'line'
frame_id: 'gripper_r_base'
visible: true
color: [0.0, 1.0, 0.0, 0.9]
parameters: [0, -1, 0, 0, 0, 0]"
