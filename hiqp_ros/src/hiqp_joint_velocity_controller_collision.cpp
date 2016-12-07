#include <pluginlib/class_list_macros.h> // to allow the controller to be loaded as a plugin

#include <hiqp_ros/hiqp_joint_velocity_controller_collision.h>
#include <hiqp_collision_check/sdf_collision_checker.h>

//#include <geometry_msgs/PoseStamped.h> // teleoperation magnet sensors

namespace hiqp_ros
{

void HiQPJointVelocityControllerCollision::initialize() {
    //call parent
    HiQPJointVelocityController::initialize();
    //create a new sdf collsion checker
    ROS_INFO("Initializing collision checker");
    collisionChecker = std::make_shared<hiqp::SDFCollisionCheck> ();
}

}

// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(hiqp_ros::HiQPJointVelocityControllerCollision, controller_interface::ControllerBase)
