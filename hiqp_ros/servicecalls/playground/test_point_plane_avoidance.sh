#disable gravity in Gazebo - useful to avoid drifting due to imperfect velocity tracking control
rosservice call /gazebo/set_physics_properties '{time_step: 0.001, max_update_rate: 1000, gravity: [0.0, 0.0, 0.0], ode_config: {auto_disable_bodies: false, sor_pgs_precon_iters: 0, sor_pgs_iters: 50, sor_pgs_w: 1.3, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2,  max_contacts: 20} }'

rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'gripper_r_base_avoidance_point'
type: 'point'
frame_id: 'gripper_r_base'
visible: true
color: [1.0, 0.0, 0.0, 0.6]
parameters: [0.0, 0.0, 0.18]"

rosservice call /yumi/hiqp_joint_velocity_controller/add_primitive \
"name: 'floor_avoidance_plane'
type: 'plane'
frame_id: 'yumi_body'
visible: true
color: [0.5, 0.5, 0.5, 1.0]
parameters: [0.0, 0.0, 1.0, 0.02]"

rosservice call /yumi/hiqp_joint_velocity_controller/set_task \
"name: 'yumi_r_gripper_floor_avoidance'
priority: 1
visible: 1
active: 1
def_params: ['TDefGeomProj', 'point', 'plane', 'gripper_r_base_avoidance_point > floor_avoidance_plane']
dyn_params: ['TDynFirstOrder', '1.0']"
