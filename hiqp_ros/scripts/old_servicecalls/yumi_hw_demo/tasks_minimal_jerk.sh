rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'ee_point_under_topplane'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsMinimalJerk', '10.0']
priority: 3
visibility: 0
parameters: ['point', 'plane', 'yumi_r_gripper_ee_point < topplane']"




rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'ee_point_on_thecylinder'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsMinimalJerk', '10.0']
priority: 3
visibility: 0
parameters: ['point', 'cylinder', 'yumi_r_gripper_ee_point = thecylinder']"





rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'ee_with_alignto_thecylinder'
type: 'TaskGeometricAlignment'
behaviour: ['DynamicsMinimalJerk', '10.0']
priority: 4
visibility: 0
parameters: ['line', 'cylinder', 'yumi_r_gripper_ee_line = thecylinder', '0']"