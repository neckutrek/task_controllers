

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'yumi_joint_limits_r1'
type: 'TaskJntLimits'
behaviour: ['DynamicsJntLimits']
priority: 1
visibility: 0
parameters: ['yumi_link_1_r', '1.4835', '2.94', '-2.94']"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'yumi_joint_limits_r2'
type: 'TaskJntLimits'
behaviour: ['DynamicsJntLimits']
priority: 1
visibility: 0
parameters: ['yumi_link_2_r', '1.4835', '0.75', '-2.5']"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'yumi_joint_limits_r3'
type: 'TaskJntLimits'
behaviour: ['DynamicsJntLimits']
priority: 1
visibility: 0
parameters: ['yumi_link_3_r', '1.7453', '2.94', '-2.94']"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'yumi_joint_limits_r4'
type: 'TaskJntLimits'
behaviour: ['DynamicsJntLimits']
priority: 1
visibility: 0
parameters: ['yumi_link_4_r', '1.3090', '-2.16', '1.4']"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'yumi_joint_limits_r5'
type: 'TaskJntLimits'
behaviour: ['DynamicsJntLimits']
priority: 1
visibility: 0
parameters: ['yumi_link_5_r', '2.2689', '5.07', '-5.07']"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'yumi_joint_limits_r6'
type: 'TaskJntLimits'
behaviour: ['DynamicsJntLimits']
priority: 1
visibility: 0
parameters: ['yumi_link_6_r', '2.2689', '2.41', '-1.54']"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'yumi_joint_limits_r7'
type: 'TaskJntLimits'
behaviour: ['DynamicsJntLimits']
priority: 1
visibility: 0
parameters: ['yumi_link_7_r', '2.2689', '4.0', '-4.0']"









# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'yumi_joint_limits_l1'
# type: 'TaskJntLimits'
# behaviour: ['DynamicsJntLimits']
# priority: 1
# visibility: 0
# parameters: ['yumi_link_1_l', '1.4835', '2.94', '-2.94']"

# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'yumi_joint_limits_l2'
# type: 'TaskJntLimits'
# behaviour: ['DynamicsJntLimits']
# priority: 1
# visibility: 0
# parameters: ['yumi_link_2_l', '1.4835', '0.75', '-2.5']"

# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'yumi_joint_limits_l3'
# type: 'TaskJntLimits'
# behaviour: ['DynamicsJntLimits']
# priority: 1
# visibility: 0
# parameters: ['yumi_link_3_l', '1.7453', '2.94', '-2.94']"

# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'yumi_joint_limits_l4'
# type: 'TaskJntLimits'
# behaviour: ['DynamicsJntLimits']
# priority: 1
# visibility: 0
# parameters: ['yumi_link_4_l', '1.3090', '-2.16', '1.4']"

# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'yumi_joint_limits_l5'
# type: 'TaskJntLimits'
# behaviour: ['DynamicsJntLimits']
# priority: 1
# visibility: 0
# parameters: ['yumi_link_5_l', '2.2689', '5.07', '-5.07']"

# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'yumi_joint_limits_l6'
# type: 'TaskJntLimits'
# behaviour: ['DynamicsJntLimits']
# priority: 1
# visibility: 0
# parameters: ['yumi_link_6_l', '2.2689', '2.41', '-1.54']"

# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'yumi_joint_limits_l7'
# type: 'TaskJntLimits'
# behaviour: ['DynamicsJntLimits']
# priority: 1
# visibility: 0
# parameters: ['yumi_link_7_l', '2.2689', '4.0', '-4.0']"