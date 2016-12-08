#ifndef HIQP_SDF_COLLISION_CHECK_HH
#define HIQP_SDF_COLLISION_CHECK_HH

#include <hiqp_collision_check/checker_base.h>
#include <ros/ros.h>
#include <hiqp_collision_check/SDFMap.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/thread/mutex.hpp>

namespace hiqp {

    class SDFCollisionCheck : public CollisionCheckerBase {

	///members
	private:
	    ros::Subscriber sdf_map_sub_;
	    //points to home handle for the node that launched us
	    ros::NodeHandle nh_;
	    //points to root node handle
	    ros::NodeHandle n_;
	    //where to listen to for new maps
	    std::string sdf_map_topic;
	    //for getting into the correct frame
	    tf::TransformListener tl;

	    boost::mutex buffer_mutex, data_mutex;
	    ///two 3d arrays of floats: one for the current map and one for receive buffer
	    //float ***grid, ***grid_buffer;
	    ///implements double buffering
	    float ****myGrid_;
	    ///sets to true once we have a first valid map
	    bool validMap;

	    std::string request_frame_id;
	    Eigen::Affine3d request2map;

	    ///metadata from message
	    std::string map_frame_id;
	    double resolution;
	    double Wmax;
	    double Dmax;
	    double Dmin;
	    int XSize, YSize, ZSize;

	///methods
	private:
	    void mapCallback(const hiqp_collision_check::SDFMap::ConstPtr& msg);  
	    ///returns the trilinear interpolated SDF value at location
	    double SDF(const Eigen::Vector3d &location);
	    /// Checks the validity of the gradient of the SDF at the current point   
	    bool ValidGradient(const Eigen::Vector3d &location);
	    /// Computes the gradient of the SDF at the location, along dimension dim, with central differences. 
	    virtual double SDFGradient(const Eigen::Vector3d &location, int dim);

	    //debug: save to vti
	    //void SaveSDF(const std::string &filename);

	public:
	    virtual bool obstacleGradient (const Eigen::Vector3d &x, Eigen::Vector3d &g, std::string frame_id="");
	    virtual bool obstacleGradientBulk (const CollisionCheckerBase::SamplesVector &x, CollisionCheckerBase::SamplesVector &g, std::string frame_id="");
	    virtual void init();
	    /// is the gradient smaller than the truncation size?
	    virtual bool isValid (const Eigen::Vector3d &grad) {
		return (!std::isnan(grad(0)+grad(1)+grad(2))); 
/*		return (grad(0) > Dmin && grad(0) < Dmax &&
			grad(1) > Dmin && grad(1) < Dmax &&
			grad(2) > Dmin && grad(2) < Dmax); 
*/
	    }
	    SDFCollisionCheck();
	    virtual ~SDFCollisionCheck();


    };

}

#endif
