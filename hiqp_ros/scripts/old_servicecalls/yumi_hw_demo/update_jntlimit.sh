# for example: 
# ./update_jntlimit 7_r 2.8 3 3

rosservice call /yumi/hiqp_kinematics_controller/update_task \
"name: 'yumi_link_$1_jntlimits'
type: 'TaskJntLimits'
behaviour: ['DynamicsJntLimits', '$2']
priority: 1
visibility: 0
parameters: ['yumi_link_$1', '$2', '$3', '$4']"