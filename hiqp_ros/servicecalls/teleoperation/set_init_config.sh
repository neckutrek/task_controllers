#disable gravity in Gazebo - useful to avoid drifting due to imperfect velocity tracking control
rosservice call /gazebo/set_physics_properties '{time_step: 0.001, max_update_rate: 1000, gravity: [0.0, 0.0, 0.0], ode_config: {auto_disable_bodies: false, sor_pgs_precon_iters: 0, sor_pgs_iters: 50, sor_pgs_w: 1.3, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2,  max_contacts: 20} }'

rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'task_teleop_init'
priority: 3
visible: 1
active: 1
def_params: ['TDefFullPose', '-1.8', '0.26', '-1.15', '0.72', '0.0', '0.0', '0.0', '0.0', '0.0', '0.42', '-1.48', '-1.21', '0.60', '1.59', '0.74', '1.39', '0.0', '0.0']
dyn_params: ['TDynFirstOrder', '1']"
