rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'jnt_config_task'
type: 'TaskJntConfig'
behaviour: ['DynamicsFirstOrder', '5.0']
priority: 1
visibility: 0
parameters: ['yumi_link_6_r', '1.0']"