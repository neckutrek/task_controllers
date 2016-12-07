rosservice call /yumi/hiqp_joint_velocity_controller/remove_task \
"task_name: 'align_gripper_vertical_axis_object_vertical_axis'"

rosservice call /yumi/hiqp_kinematics_controller/remove_task \
"task_name: 'project_gripper_approach_axis_object_vertical_axis'"

rosservice call /yumi/hiqp_kinematics_controller/remove_task \
"task_name: 'task_teleop_frame_projection'"

rosservice call /yumi/hiqp_kinematics_controller/remove_task \
"task_name: 'project_gripper_approach_axis_object_vertical_axis'"


