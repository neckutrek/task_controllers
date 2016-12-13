rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'mypoint1'
type: 'point'
frame_id: 'gripper_r_base'
visible: true
color: [0.0, 0.0, 1.0, 0.9]
parameters: [0, 0, 0.2]"

rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'mysphere1'
type: 'sphere'
frame_id: 'gripper_r_base'
visible: true
color: [0.0, 0.0, 1.0, 0.5]
parameters: [0, 0, 0.2, 0.1]"

rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'myplane2'
type: 'plane'
frame_id: 'yumi_body'
visible: true
color: [0.0, 1.0, 0.0, 0.9]
parameters: [0.0, 0, 1, 0.3]"

sleep 1.0

rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'task_test_sphere_avoid_plane1'
priority: 3
visible: 1
active: 1
def_params: ['TDefGeomProj', 'sphere', 'plane', 'mysphere1 > myplane2']
dyn_params: ['TDynFirstOrder', '1.0']"

rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'task_test_sphere_avoid_plane2'
priority: 4
visible: 1
active: 1
def_params: ['TDefGeomProj', 'point', 'plane', 'mypoint1 = myplane2']
dyn_params: ['TDynFirstOrder', '1.0']"