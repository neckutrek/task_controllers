# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'geomalignment1'
# type: 'TaskGeometricAlignment'
# behaviour: ['DynamicsFirstOrder', '0.5']
# priority: 1
# visibility: 0
# parameters: ['line', 'line', 'myline1 = myline2', '0']"

# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'geomalign2'
# type: 'TaskGeometricAlignment'
# behaviour: ['DynamicsFirstOrder', '0.5']
# priority: 1
# visibility: 0
# parameters: ['line', 'plane', 'myline1 = myplane2', '0']"

# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'geomalign3'
# type: 'TaskGeometricAlignment'
# behaviour: ['DynamicsFirstOrder', '0.5']
# priority: 1
# visibility: 0
# parameters: ['line', 'cylinder', 'myline1 = mycylinder2', '0']"

rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'geomalign4'
type: 'TaskGeometricAlignment'
behaviour: ['DynamicsFirstOrder', '0.5']
priority: 1
visibility: 0
parameters: ['line', 'sphere', 'myline1 = mysphere2', '0']"