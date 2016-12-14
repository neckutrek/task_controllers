rosservice call /yumi/hiqp_joint_velocity_controller/remove_task "task_name: 'align_gripper_vertical_axis_object_vertical_axis'" 
rosservice call /yumi/hiqp_joint_velocity_controller/remove_task "task_name: 'project_gripper_approach_axis_object_vertical_axis'" 
rosservice call /yumi/hiqp_joint_velocity_controller/remove_primitive "name: 'object_vertical_axis'" 
rosservice call /yumi/hiqp_joint_velocity_controller/remove_primitive "name: 'gripper_approach_axis'" 
rosservice call /yumi/hiqp_joint_velocity_controller/remove_primitive "name: 'gripper_vertical_axis'"


