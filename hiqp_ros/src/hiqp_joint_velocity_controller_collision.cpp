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
    collisionChecker->init();

    vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>( "gradient_marker", 10, true );
    ctr = 0;
    nh_ = ros::NodeHandle("~");

}

void HiQPJointVelocityControllerCollision::setJointControls(Eigen::VectorXd& u) {
    HiQPJointVelocityController::setJointControls(u);
    
    // for testing, ask a random point within map for a gradient
    Eigen::Vector3d pt;
    pt<<0,(double)ctr/1000. - 1,0;
    ctr = ctr<2000 ? ctr++ : 0;

    visualization_msgs::Marker pt_marker, g_marker;

    pt_marker.ns = "points"; 
    pt_marker.header.frame_id = "sdf_map_frame";
    pt_marker.header.stamp = ros::Time::now();
    pt_marker.type = visualization_msgs::Marker::SPHERE;
    pt_marker.action = visualization_msgs::Marker::ADD;
    pt_marker.id = marker_array.markers.size();
    pt_marker.pose.position.x = pt(0);
    pt_marker.pose.position.y = pt(1);
    pt_marker.pose.position.z = pt(2);
    pt_marker.pose.orientation.x = 0.0;
    pt_marker.pose.orientation.y = 0.0;
    pt_marker.pose.orientation.z = 0.0;
    pt_marker.pose.orientation.w = 1.0;
    pt_marker.scale.x = 0.005;
    pt_marker.scale.y = 0.005;
    pt_marker.scale.z = 0.005;
    pt_marker.color.r = 255;
    pt_marker.color.g = 0;
    pt_marker.color.b = 0;
    pt_marker.color.a = 0.5;
    marker_array.markers.push_back(pt_marker);


    Eigen::Vector3d grad;
    if(collisionChecker->obstacleGradient(pt,grad)) {
	if(collisionChecker->isValid(grad)) {
	    std::cerr<<grad.transpose()<<std::endl;
	    //marker_array.markers.push_back(g_marker);
	}
    }

    if(ctr == 1999) {
	vis_pub_.publish( marker_array );
	marker_array.markers.clear();
    }	

}

}

// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(hiqp_ros::HiQPJointVelocityControllerCollision, controller_interface::ControllerBase)
