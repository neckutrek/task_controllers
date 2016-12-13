

### OBSTACLE AVOIDANCE ###





## Keep the spheres outside of each other!
rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'yumi_link_5_r_l_avoidance'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsFirstOrder', '10']
priority: 1
visibility: 0
parameters: ['sphere', 'sphere', 'yumi_link_5_l_avoidance_sphere > yumi_link_5_r_avoidance_sphere']"





## Keep the spheres outside of each other!
rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'yumi_link_6_r_l_avoidance'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsFirstOrder', '10']
priority: 1
visibility: 0
parameters: ['sphere', 'sphere', 'yumi_link_6_l_avoidance_sphere > yumi_link_6_r_avoidance_sphere']"




## Keep the spheres above the plane

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'yumi_link_5_l_floor_avoidance'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsFirstOrder', '10']
priority: 1
visibility: 0
parameters: ['sphere', 'plane', 'yumi_link_5_l_avoidance_sphere > floor_avoidance_plane']"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'yumi_link_6_l_floor_avoidance'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsFirstOrder', '10']
priority: 1
visibility: 0
parameters: ['sphere', 'plane', 'yumi_link_6_l_avoidance_sphere > floor_avoidance_plane']"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'yumi_link_5_r_floor_avoidance'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsFirstOrder', '10']
priority: 1
visibility: 0
parameters: ['sphere', 'plane', 'yumi_link_5_r_avoidance_sphere > floor_avoidance_plane']"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'yumi_link_6_r_floor_avoidance'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsFirstOrder', '10']
priority: 1
visibility: 0
parameters: ['sphere', 'plane', 'yumi_link_6_r_avoidance_sphere > floor_avoidance_plane']"




## Keep the gripper obst avoid points above the plane !

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'yumi_r_gripper_floor_avoidance_1'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsFirstOrder', '10']
priority: 1
visibility: 0
parameters: ['point', 'plane', 'gripper_r_base_avoidance_point_1 > floor_avoidance_plane']"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'yumi_r_gripper_floor_avoidance_2'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsFirstOrder', '10']
priority: 1
visibility: 0
parameters: ['point', 'plane', 'gripper_r_base_avoidance_point_2 > floor_avoidance_plane']"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'yumi_r_gripper_floor_avoidance_3'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsFirstOrder', '10']
priority: 1
visibility: 0
parameters: ['point', 'plane', 'gripper_r_base_avoidance_point_3 > floor_avoidance_plane']"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'yumi_r_gripper_floor_avoidance_4'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsFirstOrder', '10']
priority: 1
visibility: 0
parameters: ['point', 'plane', 'gripper_r_base_avoidance_point_4 > floor_avoidance_plane']"




rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'yumi_l_gripper_floor_avoidance_1'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsFirstOrder', '10']
priority: 1
visibility: 0
parameters: ['point', 'plane', 'gripper_l_base_avoidance_point_1 > floor_avoidance_plane']"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'yumi_l_gripper_floor_avoidance_2'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsFirstOrder', '10']
priority: 1
visibility: 0
parameters: ['point', 'plane', 'gripper_l_base_avoidance_point_2 > floor_avoidance_plane']"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'yumi_l_gripper_floor_avoidance_3'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsFirstOrder', '10']
priority: 1
visibility: 0
parameters: ['point', 'plane', 'gripper_l_base_avoidance_point_3 > floor_avoidance_plane']"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'yumi_l_gripper_floor_avoidance_4'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsFirstOrder', '10']
priority: 1
visibility: 0
parameters: ['point', 'plane', 'gripper_l_base_avoidance_point_4 > floor_avoidance_plane']"





