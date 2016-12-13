rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'geomproj1'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsFirstOrder', '1']
priority: 3
visibility: 0
active: 1
parameters: ['point', 'point', 'teleop_point = teleop_point2']"
