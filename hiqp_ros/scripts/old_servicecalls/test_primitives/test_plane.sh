#
# Available types and parameter list syntaxes
# point:        [x, y, z]
# line:         [x,   y,  z, nx, ny, nz, l]
# line:         [x1, y1, z1, x2, y2, z2]
# plane:        [nx, ny, nz, d]
# box:          [x1, y1, z1, x2, y2, z2]
# box:          [x1, y1, z1, x2, y2, z2, nx, ny, nz, angle]
# cylinder:     [nx, ny, nz,  x,  y,  z, height, radius]
# cylinder:     [x1, y1, z1, x2, y2, z2, radius]
# sphere:       [x, y, z, radius]
#

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'myplane1'
type: 'plane'
frame_id: 'yumi_body'
visible: true
color: [1, 1, 0, 0.4]
parameters: [1, 1, 0, 1]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'myplane2'
type: 'plane'
frame_id: 'yumi_body'
visible: true
color: [1, 0, 1, 0.4]
parameters: [1, 0, 1, 1]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'myplane3'
type: 'plane'
frame_id: 'yumi_body'
visible: true
color: [0, 1, 1, 0.4]
parameters: [0, 1, 1, 1]"




rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'myplane4'
type: 'plane'
frame_id: 'yumi_body'
visible: true
color: [1, 1, 0, 0.4]
parameters: [-1, -1, 0, 1]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'myplane5'
type: 'plane'
frame_id: 'yumi_body'
visible: true
color: [1, 0, 1, 0.4]
parameters: [-1, 0, -1, 1]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'myplane6'
type: 'plane'
frame_id: 'yumi_body'
visible: true
color: [0, 1, 1, 0.4]
parameters: [0, -1, -1, 1]"