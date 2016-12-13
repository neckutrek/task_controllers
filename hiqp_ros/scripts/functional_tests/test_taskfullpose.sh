rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'task_test_taskfullpose'
priority: 3
visible: 1
active: 1
def_params: ['TDefFullPose', '-0.42', '-1.48', '1.21', '0.75', '-0.8', '0.45', '1.21', '0.42', '-1.48', '-1.21', '0.75', '0.8', '0.45', '1.21']
dyn_params: ['TDynFirstOrder', '0.1']"