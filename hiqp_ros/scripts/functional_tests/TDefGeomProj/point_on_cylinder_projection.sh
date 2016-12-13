rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'mypoint1'
type: 'point'
frame_id: 'gripper_r_base'
visible: true
color: [0.0, 0.0, 1.0, 0.9]
parameters: [0.0, 0, 0.1]"

rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'mycylinder2'
type: 'cylinder'
frame_id: 'yumi_body'
visible: true
color: [0.0, 1.0, 0.0, 0.9]
parameters: [0, 0, 1, 0.5, 0, 0, 0.06, 0.2]"

sleep 1.0

rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'task_test_taskgeomproj_point_cylinder'
priority: 3
visible: 1
active: 1
def_params: ['TDefGeomProj', 'point', 'cylinder', 'mypoint1 = mycylinder2']
dyn_params: ['TDynFirstOrder', '1.0']"