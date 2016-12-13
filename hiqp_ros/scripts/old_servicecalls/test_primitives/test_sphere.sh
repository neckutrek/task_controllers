#
# Available types and parameter list syntaxes
# point:        [x, y, z]
# line:         [nx, ny, nz,  x,  y,  z, l]
# line:         [x1, y1, z1, x2, y2, z2]
# plane:        [x, y, z, nx, ny, nz]
# box:          [x, y, z, w, d, h, nx, ny, nz, a]
# cylinder:     [nx, ny, nz,  x,  y,  z, height, radius]
# cylinder:     [x1, y1, z1, x2, y2, z2, radius]
# sphere:       [x, y, z, radius]
#

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'mysphere1'
type: 'sphere'
frame_id: 'yumi_body'
visible: true
color: [0.9, 0.9, 0.0, 0.9]
parameters: ['0.4', '0.4', '0', '0.1']"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'mysphere2'
type: 'sphere'
frame_id: 'yumi_body'
visible: true
color: [0.9, 0.0, 0.9, 0.9]
parameters: ['0.4', '0', '0.4', '0.1']"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'mysphere3'
type: 'sphere'
frame_id: 'yumi_body'
visible: true
color: [0.0, 0.9, 0.9, 0.9]
parameters: ['0', '0.4', '0.4', '0.1']"




rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'mysphere4'
type: 'sphere'
frame_id: 'yumi_body'
visible: true
color: [0.9, 0.9, 0.0, 0.9]
parameters: ['-0.4', '-0.4', '0', '0.1']"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'mysphere5'
type: 'sphere'
frame_id: 'yumi_body'
visible: true
color: [0.9, 0.0, 0.9, 0.9]
parameters: ['-0.4', '0', '-0.4', '0.1']"

rosservice call /yumi/hiqp_kinematics_controller/add_primitive \
"name: 'mysphere6'
type: 'sphere'
frame_id: 'yumi_body'
visible: true
color: [0.0, 0.9, 0.9, 0.9]
parameters: ['0', '-0.4', '-0.4', '0.1']"