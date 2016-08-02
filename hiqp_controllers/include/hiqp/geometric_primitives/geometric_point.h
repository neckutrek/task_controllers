// The HiQP Control Framework, an optimal control framework targeted at robotics
// Copyright (C) 2016 Marcus A Johansson
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.




/*!
 * \file   geometric_point.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_GEOMETRIC_POINT_H
#define HIQP_GEOMETRIC_POINT_H



#include <hiqp/geometric_primitives/geometric_primitive.h>

#include <kdl/frames.hpp>

#include <Eigen/Dense>




namespace hiqp
{




/*!
 * \class GeometricPoint
 * \brief 
 */  
class GeometricPoint : public GeometricPrimitive
{
public:

	/*!
     * \brief Constructor
     */
	GeometricPoint
	(
		const std::string& name,
		const std::string& frame_id,
		bool visible,
		const std::vector<double>& color,
		const std::vector<std::string>& parameters
	)
	: GeometricPrimitive(name, frame_id, visible, color)
	{
		assert(parameters.size() == 3);

		kdl_p_(0) = std::stod( parameters.at(0) );
		kdl_p_(1) = std::stod( parameters.at(1) );
		kdl_p_(2) = std::stod( parameters.at(2) );

		eigen_p_ << kdl_p_(0), kdl_p_(1), kdl_p_(2);
	}



	/*!
     * \brief Destructor
     */
	~GeometricPoint() noexcept {}



	inline const KDL::Vector&       getPointKDL() 
	{ return kdl_p_; }

	inline const Eigen::Vector3d&   getPointEigen() 
	{ return eigen_p_; }


	inline double getX() { return kdl_p_(0); }
	inline double getY() { return kdl_p_(1); }
	inline double getZ() { return kdl_p_(2); }



protected:

	KDL::Vector    		kdl_p_;
	Eigen::Vector3d		eigen_p_;



private:

	// No copying of this class is allowed !
	GeometricPoint(const GeometricPoint& other) = delete;
	GeometricPoint(GeometricPoint&& other) = delete;
	GeometricPoint& operator=(const GeometricPoint& other) = delete;
	GeometricPoint& operator=(GeometricPoint&& other) noexcept = delete;



};











} // namespace hiqp

#endif // include guard