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



# rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
# "name: 'myline2'
# type: 'line'
# frame_id: 'gripper_l_base'
# visible: true
# color: [1.0, 1.0, 1.0, 0.9]
# parameters: [0.0, 0.1, 1, 0, -0.1, 0]"

# rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
# "name: 'myline22'
# type: 'line'
# frame_id: 'yumi_body'
# visible: true
# color: [1.0, 1.0, 1.0, 0.9]
# parameters: [0.0, 0.0, 1, 0.4, 0, 0]"



# rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
# "name: 'myplane2'
# type: 'plane'
# frame_id: 'gripper_l_base'
# visible: true
# color: [0.6, 0.6, 1.0, 0.4]
# parameters: [1, 0, 0, -0.3]"


#rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
#"name: 'mybox1'
#type: 'box'
#frame_id: 'yumi_body'
#visible: true
#color: [0.6, 0.6, 1.0, 0.6]
#parameters: [0.5, 0, 0.05, 0.1, 0.1, 0.1, 0.785, 0.785, 0.785]"


#rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
#"name: 'mycylinder2'
#type: 'cylinder'
#frame_id: 'yumi_body'
#visible: true
#color: [0.6, 1.0, 0.6, 0.7]
#parameters: [0, 0, 1, 0.4, 0, 0, 0.3, 0.4]"


# rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
# "name: 'mysphere2'
# type: 'sphere'
# frame_id: 'yumi_body'
# visible: true
# color: [1.0, 0.6, 0.6, 0.7]
# parameters: [0.4, 0, 0, 0.4]"
