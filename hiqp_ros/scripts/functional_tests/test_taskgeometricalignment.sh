rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'myline1'
type: 'line'
frame_id: 'gripper_l_base'
visible: true
color: [1.0, 0.0, 0.0, 0.9]
parameters: [0, 0, -1, 0, 0, 0]"

rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'myline2'
type: 'line'
frame_id: 'gripper_r_base'
visible: true
color: [0.0, 1.0, 0.0, 0.9]
parameters: [0, 0, 1, 0, 0, 0]"

sleep 1.0

rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'task_test_taskjntconfig'
priority: 3
visible: 1
active: 1
def_params: ['TDefGeomAlign', 'line', 'line', 'myline1 = myline2', '0.0']
dyn_params: ['TDynFirstOrder', 1.0']"