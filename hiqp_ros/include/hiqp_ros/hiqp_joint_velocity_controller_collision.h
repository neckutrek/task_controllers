#ifndef HIQP_JOINT_VELOCITY_CONTROLLER_COLLISION_H
#define HIQP_JOINT_VELOCITY_CONTROLLER_COLLISION_H

#include <hiqp_ros/hiqp_joint_velocity_controller.h>
#include <hiqp_collision_check/checker_base.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

namespace hiqp_ros {

class HiQPJointVelocityControllerCollision : public HiQPJointVelocityController {

    public:
	void initialize();
	void computeControls(Eigen::VectorXd& u);

	void starting(const ros::Time& time) {
	    if(collisionChecker!=nullptr) {
		collisionChecker->activate();
	    }
	}
	void stopping(const ros::Time& time) {
	    if(collisionChecker!=nullptr) {
		collisionChecker->deactivate();
	    }
	}

    protected:
	std::shared_ptr<hiqp::CollisionCheckerBase> collisionChecker;
	////debug stuff////
	int ctr;
	ros::NodeHandle nh_;
	ros::Publisher vis_pub_;	
	visualization_msgs::MarkerArray marker_array;

	hiqp::CollisionCheckerBase::SamplesVector samples, gradients;
};
    
}

#endif
