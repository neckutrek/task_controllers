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

    nh_ = this->getControllerNodeHandle();
    vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>( "gradient_marker", 1 );
    ctr = 0;

}

void HiQPJointVelocityControllerCollision::setJointControls(Eigen::VectorXd& u) {
    HiQPJointVelocityController::setJointControls(u);
    
    // for testing, ask a random point within map for a gradient
    Eigen::Vector3d pt;
    pt<<0,(double)ctr/1000. - 1,-0.03;

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
    ctr = ctr<2000 ? ctr+1 : 0;


    Eigen::Vector3d grad;
    if(collisionChecker->obstacleGradient(pt,grad)) {
	if(collisionChecker->isValid(grad)) {
	    std::cerr<<grad.transpose()<<std::endl;
	    //normal
	    geometry_msgs::Point start, end;

	    //grad = 10*grad;
	    start.x = pt(0);
	    start.y = pt(1);
	    start.z = pt(2);
	    end.x = pt(0) + grad(0);
	    end.y = pt(1) + grad(1);
	    end.z = pt(2) + grad(2);
	    //Eigen::Quaterniond q;
	    //q.setFromTwoVectors(Eigen::Vector3d::UnitX() , grad);

	    g_marker.header.frame_id = "sdf_map_frame";
	    g_marker.header.stamp = ros::Time::now();
	    g_marker.ns = "gradient";
	    g_marker.type =  visualization_msgs::Marker::ARROW;
	    g_marker.action = visualization_msgs::Marker::ADD;
	    g_marker.id = marker_array.markers.size();
	    /*g_marker.pose.position.x = pt(0);
	    g_marker.pose.position.y = pt(1);
	    g_marker.pose.position.z = pt(2);
	    g_marker.pose.orientation.x = q.x();
	    g_marker.pose.orientation.y = q.y();
	    g_marker.pose.orientation.z = q.z();
	    g_marker.pose.orientation.w = q.w(); */
	    g_marker.scale.x = 0.003; 
	    g_marker.scale.y = 0.005;
	    g_marker.scale.z = 0.005;
	    g_marker.points.push_back(start);
	    g_marker.points.push_back(end);
	    g_marker.color.r = 1.0;
	    g_marker.color.g = 0.0;
	    g_marker.color.b = 1.0;
	    g_marker.color.a = 1.0;

	    marker_array.markers.push_back(g_marker);
	}
    }

    if(ctr == 1999) {
	for(int i=0; i<marker_array.markers.size(); i++) {
	    marker_array.markers[i].header.stamp = ros::Time::now();
	    marker_array.markers[i].lifetime = ros::Duration(0);
	}
	vis_pub_.publish( marker_array );
	marker_array.markers.clear();
	std::cerr<<"clear markers\n";
    }

}

}

// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(hiqp_ros::HiQPJointVelocityControllerCollision, controller_interface::ControllerBase)
