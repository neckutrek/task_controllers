#ifndef HIQP_COLLISION_BASE_HH
#define HIQP_COLLISION_BASE_HH

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>

//#include <boost/thread/
namespace hiqp {

    /**
      * @brief base class for all collision checkers. 
      * Provides methods to initialize and start a background thread for model maintenance.
      * Provides interfaces for answering collision querries
      */
    class CollisionCheckerBase {
	public:
	    typedef std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > SamplesVector;

	    virtual bool obstacleGradient (const Eigen::Vector3d &x, Eigen::Vector3d &g, std::string frame_id="") = 0;
	    virtual bool obstacleGradientBulk (const SamplesVector &x, SamplesVector &g, std::string frame_id="") = 0;
	    virtual void init() = 0;
	    //to check if a gradient to an obstacle is valid
	    virtual bool isValid (const Eigen::Vector3d &grad) = 0; 

	    void activate() { 
		active_mutex.lock();
		isActive_ = true; 
		active_mutex.unlock();
		ROS_INFO("Collision check activated");
	    }
	    void deactivate() { 
		active_mutex.lock();
		isActive_ = false; 
		active_mutex.unlock();
		ROS_INFO("Collision check deactivated");
	    }

	    bool isActive() { 
		bool act; 
		active_mutex.lock();
		act=isActive_;
		active_mutex.unlock();
		return act;
	    }
	protected:
	    bool isActive_;
	    boost::mutex active_mutex;

    };

}

#endif
