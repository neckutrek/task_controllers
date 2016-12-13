rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'task_test_taskjntconfig2'
priority: 3
visible: 1
active: 1
def_params: ['TDefJntConfig', 'yumi_link_5_l', '-1.0']
dyn_params: ['TDynFirstOrder', '1.0']"
