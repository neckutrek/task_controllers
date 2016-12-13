#
# Available types and parameter list syntaxes
# point:        [x, y, z]
# line:         [x,   y,  z, nx, ny, nz, l]
# line:         [x1, y1, z1, x2, y2, z2]
# plane:        [x, y, z, nx, ny, nz]
# box:          [x1, y1, z1, x2, y2, z2]
# box:          [x1, y1, z1, x2, y2, z2, nx, ny, nz, angle]
# cylinder:     [nx, ny, nz,  x,  y,  z, height, radius]
# cylinder:     [x1, y1, z1, x2, y2, z2, radius]
# sphere:       [x, y, z, radius]
#

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'mypoint1'
type: 'point'
frame_id: 'yumi_body'
visible: true
color: [0.9, 0.0, 0.0, 0.9]
parameters: ['0.5', '0', '0']"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'mypoint2'
type: 'point'
frame_id: 'yumi_body'
visible: true
color: [0.0, 0.9, 0.0, 0.9]
parameters: ['0', '0.5', '0']"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'mypoint3'
type: 'point'
frame_id: 'yumi_body'
visible: true
color: [0.0, 0.0, 0.9, 0.9]
parameters: ['0', '0', '0.5']"




rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'mypoint4'
type: 'point'
frame_id: 'yumi_body'
visible: true
color: [0.9, 0.0, 0.0, 0.9]
parameters: ['-0.5', '0', '0']"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'mypoint5'
type: 'point'
frame_id: 'yumi_body'
visible: true
color: [0.0, 0.9, 0.0, 0.9]
parameters: ['0', '-0.5', '0']"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'mypoint6'
type: 'point'
frame_id: 'yumi_body'
visible: true
color: [0.0, 0.0, 0.9, 0.9]
parameters: ['0', '0', '-0.5']"