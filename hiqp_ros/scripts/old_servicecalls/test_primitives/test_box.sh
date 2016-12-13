#
# Available types and parameter list syntaxes
# point:        [x, y, z]
# line:         [nx, ny, nz,  x,  y,  z,]
# plane:        [x, y, z, nx, ny, nz]
# box:          [x1, y1, z1, x2, y2, z2]
# box:          [x1, y1, z1, x2, y2, z2, nx, ny, nz, angle]
# cylinder:     [nx, ny, nz,  x,  y,  z, radius, height]
# sphere:       [x, y, z, radius]
#




rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'mybox1'
type: 'box'
frame_id: 'yumi_body'
visible: true
color: [0.9, 0.9, 0.9, 0.6]
parameters: ['0.5', '0.5', '0.5', '1', '0.7', '0.4', '0', '0', '1']"