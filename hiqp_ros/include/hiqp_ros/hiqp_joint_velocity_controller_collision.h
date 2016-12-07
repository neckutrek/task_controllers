#ifndef HIQP_JOINT_VELOCITY_CONTROLLER_COLLISION_H
#define HIQP_JOINT_VELOCITY_CONTROLLER_COLLISION_H

#include <hiqp_ros/hiqp_joint_velocity_controller.h>
#include <hiqp_collision_check/checker_base.h>

namespace hiqp_ros {

class HiQPJointVelocityControllerCollision : public HiQPJointVelocityController {

    public:
	void initialize();
	void setJointControls(Eigen::VectorXd& u);

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
	int ctr;	

};
    
}

#endif
