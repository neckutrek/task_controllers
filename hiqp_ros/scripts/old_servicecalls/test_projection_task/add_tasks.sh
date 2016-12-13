# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'geomproj1'
# type: 'TaskGeometricProjection'
# behaviour: ['DynamicsFirstOrder', '10']
# priority: 3
# visibility: 0
# parameters: ['point', 'point', 'mypoint1 = mypoint2']"



rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'geomproj1'
type: 'TaskGeometricProjection'
behaviour: ['DynamicsMinimalJerk', '5.0']
priority: 3
visibility: 0
parameters: ['point', 'point', 'mypoint1 = mypoint2']"




# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'geomproj2'
# type: 'TaskGeometricProjection'
# behaviour: ['DynamicsFirstOrder', '10']
# priority: 1
# visibility: 0
# parameters: ['point', 'line', 'mypoint1 = myline2']"




# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'geomproj3'
# type: 'TaskGeometricProjection'
# behaviour: ['DynamicsFirstOrder', '10']
# priority: 1
# visibility: 0
# parameters: ['point', 'plane', 'mypoint1 = myplane2']"

# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'geomproj4'
# type: 'TaskGeometricProjection'
# behaviour: ['DynamicsFirstOrder', '10']
# priority: 1
# visibility: 0
# parameters: ['point', 'plane', 'mypoint1 < myplane2']"

# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'geomproj5'
# type: 'TaskGeometricProjection'
# behaviour: ['DynamicsFirstOrder', '10']
# priority: 1
# visibility: 0
# parameters: ['point', 'plane', 'mypoint1 > myplane2']"




# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'geomproj155'
# type: 'TaskGeometricProjection'
# behaviour: ['DynamicsFirstOrder', '0.5']
# priority: 1
# visibility: 0
# parameters: ['point', 'box', 'mypoint1 = mybox1']"




# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'geomproj6'
# type: 'TaskGeometricProjection'
# behaviour: ['DynamicsFirstOrder', '8.0']
# priority: 1
# visibility: 0
# parameters: ['point', 'cylinder', 'mypoint1 = mycylinder2']"

# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'geomproj7'
# type: 'TaskGeometricProjection'
# behaviour: ['DynamicsFirstOrder', '10']
# priority: 1
# visibility: 0
# parameters: ['point', 'cylinder', 'mypoint1 < mycylinder2']"

# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'geomproj8'
# type: 'TaskGeometricProjection'
# behaviour: ['DynamicsFirstOrder', '10']
# priority: 1
# visibility: 0
# parameters: ['point', 'cylinder', 'mypoint1 > mycylinder2']"




# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'geomproj9'
# type: 'TaskGeometricProjection'
# behaviour: ['DynamicsFirstOrder', '10']
# priority: 1
# visibility: 0
# parameters: ['point', 'sphere', 'mypoint1 = mysphere2']"

# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'geomproj10'
# type: 'TaskGeometricProjection'
# behaviour: ['DynamicsFirstOrder', '10']
# priority: 1
# visibility: 0
# parameters: ['point', 'sphere', 'mypoint1 < mysphere2']"

# rosservice call /yumi/hiqp_kinematics_controller/add_task \
# "name: 'geomproj11'
# type: 'TaskGeometricProjection'
# behaviour: ['DynamicsFirstOrder', '10']
# priority: 1
# visibility: 0
# parameters: ['point', 'sphere', 'mypoint1 > mysphere2']"





