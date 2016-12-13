rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'jnt_limits_task'
type: 'TaskJntLimits'
behaviour: []
priority: 1
visibility: 0
parameters: ['yumi_link_6_r', '0.3', '0.1', '0.2']"