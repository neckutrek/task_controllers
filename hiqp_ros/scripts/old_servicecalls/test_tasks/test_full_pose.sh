rosservice call /yumi/hiqp_kinematics_controller/add_task \
"name: 'full_pose_task'
type: 'TaskFullPose'
behaviour: ['DynamicsFirstOrder', '5.0']
priority: 1
visibility: 0
parameters: ['0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1', '0.1']"