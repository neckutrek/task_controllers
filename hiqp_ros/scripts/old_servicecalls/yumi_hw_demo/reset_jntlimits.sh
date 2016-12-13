rosservice call /yumi/hiqp_kinematics_controller/update_task \
"name: 'yumi_link_1_r_jntlimits'
type: 'TaskJntLimits'
behaviour: ['DynamicsJntLimits', '1.4835']
priority: 1
visibility: 0
parameters: ['yumi_link_1_r', '1.4835', '-2.94', '2.94']"

rosservice call /yumi/hiqp_kinematics_controller/update_task \
"name: 'yumi_link_2_r_jntlimits'
type: 'TaskJntLimits'
behaviour: ['DynamicsJntLimits', '1.4835']
priority: 1
visibility: 0
parameters: ['yumi_link_2_r', '1.4835', '-2.5', '0.75']"

rosservice call /yumi/hiqp_kinematics_controller/update_task \
"name: 'yumi_link_3_r_jntlimits'
type: 'TaskJntLimits'
behaviour: ['DynamicsJntLimits', '1.7453']
priority: 1
visibility: 0
parameters: ['yumi_link_3_r', '1.7453', '-2.94', '2.94']"

rosservice call /yumi/hiqp_kinematics_controller/update_task \
"name: 'yumi_link_4_r_jntlimits'
type: 'TaskJntLimits'
behaviour: ['DynamicsJntLimits', '1.3090']
priority: 1
visibility: 0
parameters: ['yumi_link_4_r', '1.3090', '-2.16', '1.4']"

rosservice call /yumi/hiqp_kinematics_controller/update_task \
"name: 'yumi_link_5_r_jntlimits'
type: 'TaskJntLimits'
behaviour: ['DynamicsJntLimits', '2.2689']
priority: 1
visibility: 0
parameters: ['yumi_link_5_r', '2.2689', '-5.07', '5.07']"

rosservice call /yumi/hiqp_kinematics_controller/update_task \
"name: 'yumi_link_6_r_jntlimits'
type: 'TaskJntLimits'
behaviour: ['DynamicsJntLimits', '2.2689']
priority: 1
visibility: 0
parameters: ['yumi_link_6_r', '2.2689', '-1.54', '2.41']"

rosservice call /yumi/hiqp_kinematics_controller/update_task \
"name: 'yumi_link_7_r_jntlimits'
type: 'TaskJntLimits'
behaviour: ['DynamicsJntLimits', '2.2689']
priority: 1
visibility: 0
parameters: ['yumi_link_7_r', '2.2689', '-4.0', '4.0']"
