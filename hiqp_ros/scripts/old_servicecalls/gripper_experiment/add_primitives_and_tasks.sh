### ADD PRIMITIVES

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'experiment_gripper_point'
type: 'point'
frame_id: 'gripper_r_base'
visible: true
color: [0.0, 0.0, 1.0, 0.6]
parameters: [0.0, 0.0, 0.1]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'experiment_starting_point'
type: 'point'
frame_id: 'world'
visible: true
color: [1.0, 0.0, 1.0, 1.0]
parameters: [0.1, 0.2, 0.2]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'experiment_gripper_line_z'
type: 'line'
frame_id: 'gripper_r_base'
visible: true
color: [0.0, 0.0, 1.0, 0.6]
parameters: [0.0, 0.0, 1.0, 0.0, 0.0, 0.1]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'experiment_gripper_line_y'
type: 'line'
frame_id: 'gripper_r_base'
visible: true
color: [1.0, 0.0, 1.0, 0.6]
parameters: [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'experiment_plane'
type: 'plane'
frame_id: 'world'
visible: true
color: [1.0, 1.0, 1.0, 0.6]
parameters: [0.0, 0.0, 1.0, 0.215]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'experiment_cylinder'
type: 'cylinder'
frame_id: 'world'
visible: true
color: [0.0, 1.0, 0.0, 0.6]
parameters: [0.0, 0.0, 1.0, 0.5, 0.0, 0.115, 0.033, 0.1]"






### ADD TASKS, make sure they're deacitvated to start with

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'bring_back_to_start'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsFirstOrder', '1']
priority: 2
visibility: 0
active: 0
parameters: ['point', 'point', 'experiment_gripper_point = experiment_starting_point']"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'bring_gripper_point_to_cylinder'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsFirstOrder', '1']
priority: 2
visibility: 0
active: 0
parameters: ['point', 'cylinder', 'experiment_gripper_point = experiment_cylinder']"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'bring_gripper_point_above_floor'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsFirstOrder', '1']
priority: 2
visibility: 0
active: 0
parameters: ['point', 'plane', 'experiment_gripper_point > floor_avoidance_plane']"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'bring_gripper_point_under_plane'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsFirstOrder', '1']
priority: 2
visibility: 0
active: 0
parameters: ['point', 'plane', 'experiment_gripper_point < experiment_plane']"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'align_gripper_with_floor'
type: 'TaskGeometricAlignment'
behaviour: ['DynamicsFirstOrder', '0.5']
priority: 3
visibility: 0
active: 0
parameters: ['line', 'plane', 'experiment_gripper_line_y = floor_avoidance_plane', '0.0']"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'align_gripper_with_cylinder'
type: 'TaskGeometricAlignment'
behaviour: ['DynamicsFirstOrder', '0.5']
priority: 4
visibility: 0
active: 0
parameters: ['line', 'cylinder', 'experiment_gripper_line_y = experiment_cylinder', '0.0']"