#
# Available types and parameter list syntaxes
# point:        [x, y, z]
# line:         [nx, ny, nz,  x,  y,  z,]
# plane:        [x, y, z, nx, ny, nz]
# box:          [x1, y1, z1, x2, y2, z2]
# box:          [x1, y1, z1, x2, y2, z2, nx, ny, nz, angle]
# cylinder:     [nx, ny, nz,  x,  y,  z, height, radius]
# cylinder:     [x1, y1, z1, x2, y2, z2, radius]
# sphere:       [x, y, z, radius]
#




rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'myline4'
type: 'line'
frame_id: 'yumi_body'
visible: true
color: [0.9, 0.9, 0.0, 0.9]
parameters: [0.1, 0.1, 0, 0.5, 0.5, 0]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'myline5'
type: 'line'
frame_id: 'yumi_body'
visible: true
color: [0.0, 0.9, 0.9, 0.9]
parameters: [0, 0.1, 0.1, 0, 0.5, 0.5]"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'myline6'
type: 'line'
frame_id: 'yumi_body'
visible: true
color: [0.9, 0.0, 0.9, 0.9]
parameters: [0.1, 0, 0.1, 0.5, 0, 0.5]"


