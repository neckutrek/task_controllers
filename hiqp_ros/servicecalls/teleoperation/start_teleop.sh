rosservice call /wintracker_rebase/synchronize 

sleep 1.0

rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'task_teleop_frame_projection'
priority: 3
visible: 1
active: 1
def_params: ['TDefGeomProj', 'frame', 'frame', 'teleop_wintracker_frame = teleop_gripper_frame']
dyn_params: ['TDynFirstOrder', '8.0']"

rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'task_teleop_frame_alignment'
priority: 3
visible: 1
active: 1
def_params: ['TDefGeomAlign', 'frame', 'frame', 'teleop_wintracker_frame = teleop_gripper_frame', '0']
dyn_params: ['TDynFirstOrder', '1.0']"



