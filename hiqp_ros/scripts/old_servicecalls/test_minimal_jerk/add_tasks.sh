rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'jntlimits1'
type: 'TaskJntConfigOne'
behaviour: ['DynamicsMinimalJerk', '5.0']
priority: 3
visibility: 0
parameters: ['yumi_link_4_r', '$1']"