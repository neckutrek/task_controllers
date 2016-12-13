rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'mytask'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsFirstOrder', '10']
priority: 2
visibility: 0
parameters: ['point', 'point', 'mypoint1 = mypoint2']"