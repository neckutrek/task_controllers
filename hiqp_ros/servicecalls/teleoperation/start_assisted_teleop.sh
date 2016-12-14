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

rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'align_gripper_vertical_axis_object_vertical_axis'
priority: 2
visible: 1
active: 1
def_params: ['TDefGeomAlign', 'line', 'line', 'object_vertical_axis = gripper_vertical_axis', '0.0']
dyn_params: ['TDynFirstOrder', 2.0']"

rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'project_gripper_approach_axis_object_vertical_axis'
priority: 2
visible: 1
active: 1
def_params: ['TDefGeomProj', 'line', 'line', 'gripper_approach_axis = object_vertical_axis']
dyn_params: ['TDynFirstOrder', '2.0']"


