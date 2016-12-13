rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'myframe1'
type: 'frame'
frame_id: 'gripper_r_base'
visible: true
color: [0, 0, 0, 1]
parameters: [0, 0, 0.1, 0, 0, 0, 1]"

rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'myframe2'
type: 'frame'
frame_id: 'gripper_l_base'
visible: true
color: [0, 0, 0, 1]
parameters: [0, 0, 0.1, 0, 0, 0, 1]"

sleep 1.0

rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'test_frame_on_frame_alignment'
priority: 3
visible: 1
active: 1
def_params: ['TDefGeomAlign', 'frame', 'frame', 'myframe1 = myframe2', '0']
dyn_params: ['TDynFirstOrder', '1.0']"