

###           world

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'floor_avoidance_plane'
type: 'plane'
frame_id: 'world'
visible: true
color: [0.2, 0.2, 1.0, 0.5]
parameters: [0.0, 0.0, 1.0, 0.115]"





###           yumi_link_5_r

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'yumi_link_5_r_avoidance_sphere'
type: 'sphere'
frame_id: 'yumi_link_5_r'
visible: true
color: [1.0, 0.0, 0.0, 0.6]
parameters: [-0.023, -0.04, 0.1, 0.04]"





###           yumi_link_6_r

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'yumi_link_6_r_avoidance_sphere'
type: 'sphere'
frame_id: 'yumi_link_6_r'
visible: true
color: [1.0, 0.0, 0.0, 0.6]
parameters: [0.01, -0.01, -0.01, 0.06]"





###           gripper_r_base

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'gripper_r_base_avoidance_point_1'
type: 'point'
frame_id: 'gripper_r_base'
visible: true
color: [1.0, 0.0, 0.0, 0.6]
parameters: [0.036, 0.035, 0.135]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'gripper_r_base_avoidance_point_2'
type: 'point'
frame_id: 'gripper_r_base'
visible: true
color: [1.0, 0.0, 0.0, 0.6]
parameters: [-0.036, 0.035, 0.135]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'gripper_r_base_avoidance_point_3'
type: 'point'
frame_id: 'gripper_r_base'
visible: true
color: [1.0, 0.0, 0.0, 0.6]
parameters: [0.036, -0.035, 0.135]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'gripper_r_base_avoidance_point_4'
type: 'point'
frame_id: 'gripper_r_base'
visible: true
color: [1.0, 0.0, 0.0, 0.6]
parameters: [-0.036, -0.035, 0.135]"





###           yumi_link_5_l

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'yumi_link_5_l_avoidance_sphere'
type: 'sphere'
frame_id: 'yumi_link_5_l'
visible: true
color: [1.0, 0.0, 0.0, 0.6]
parameters: [-0.023, -0.04, 0.1, 0.04]"





###           yumi_link_6_l

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'yumi_link_6_l_avoidance_sphere'
type: 'sphere'
frame_id: 'yumi_link_6_l'
visible: true
color: [1.0, 0.0, 0.0, 0.6]
parameters: [0.01, -0.01, -0.01, 0.06]"




###           gripper_l_base

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'gripper_l_base_avoidance_point_1'
type: 'point'
frame_id: 'gripper_l_base'
visible: true
color: [1.0, 0.0, 0.0, 0.6]
parameters: [0.036, 0.035, 0.135]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'gripper_l_base_avoidance_point_2'
type: 'point'
frame_id: 'gripper_l_base'
visible: true
color: [1.0, 0.0, 0.0, 0.6]
parameters: [-0.036, 0.035, 0.135]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'gripper_l_base_avoidance_point_3'
type: 'point'
frame_id: 'gripper_l_base'
visible: true
color: [1.0, 0.0, 0.0, 0.6]
parameters: [0.036, -0.035, 0.135]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'gripper_l_base_avoidance_point_4'
type: 'point'
frame_id: 'gripper_l_base'
visible: true
color: [1.0, 0.0, 0.0, 0.6]
parameters: [-0.036, -0.035, 0.135]"





