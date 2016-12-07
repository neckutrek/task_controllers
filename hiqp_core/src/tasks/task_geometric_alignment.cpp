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

/*
 * \file   task_geometric_alignment.cpp
 * \author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */

#include <hiqp/tasks/task_geometric_alignment.h>

#include <hiqp/geometric_primitives/geometric_point.h>
#include <hiqp/geometric_primitives/geometric_line.h>
#include <hiqp/geometric_primitives/geometric_plane.h>
#include <hiqp/geometric_primitives/geometric_box.h>
#include <hiqp/geometric_primitives/geometric_cylinder.h>
#include <hiqp/geometric_primitives/geometric_sphere.h>
#include <hiqp/geometric_primitives/geometric_frame.h>

#include <hiqp/utilities.h>

#include <iostream>
#include <cmath>





namespace hiqp
{
namespace tasks
{





template<>
int TaskGeometricAlignment<GeometricLine, GeometricLine>::align
(
	std::shared_ptr<GeometricLine> line1,
	std::shared_ptr<GeometricLine> line2
)
{
	KDL::Vector v1 = pose_a_.M * line1->getDirectionKDL();
	KDL::Vector v2 = pose_b_.M * line2->getDirectionKDL();

	return alignVectors(v1, v2);
}





template<>
int TaskGeometricAlignment<GeometricLine, GeometricPlane>::align
(
	std::shared_ptr<GeometricLine> line,
	std::shared_ptr<GeometricPlane> plane
)
{
	KDL::Vector v1 = pose_a_.M * line->getDirectionKDL();
	KDL::Vector v2 = - ( pose_b_.M * plane->getNormalKDL() );

	return alignVectors(v1, v2);
}





template<>
int TaskGeometricAlignment<GeometricLine, GeometricCylinder>::align
(
	std::shared_ptr<GeometricLine> line,
	std::shared_ptr<GeometricCylinder> cylinder
)
{
	KDL::Vector v1 = - (pose_a_.M * line->getDirectionKDL());

	KDL::Vector p = pose_a_.p + pose_a_.M * line->getOffsetKDL();
	KDL::Vector d = pose_b_.p + pose_b_.M * cylinder->getOffsetKDL();
	KDL::Vector v = pose_b_.M * cylinder->getDirectionKDL();

	KDL::Vector x = KDL::dot( (p-d), v) * v;

	KDL::Vector v2 = (p-d) - x;

	v2.Normalize();

	return alignVectors(v1, v2);
}





template<>
int TaskGeometricAlignment<GeometricLine, GeometricSphere>::align
(
	std::shared_ptr<GeometricLine> line,
	std::shared_ptr<GeometricSphere> sphere
)
{
	KDL::Vector v1 = pose_a_.M * line->getDirectionKDL();

	KDL::Vector p = pose_a_.p + pose_a_.M * line->getOffsetKDL();
	KDL::Vector d = pose_b_.p + pose_b_.M * sphere->getCenterKDL();

	KDL::Vector v2 = d - p;

	v2.Normalize();

	return alignVectors(v1, v2);
}



/// \bug Frame alignment seems not to consider the axis sense (e.g., alignment along x and -x seems the same)
template<>
int TaskGeometricAlignment<GeometricFrame, GeometricFrame>::align
(
	std::shared_ptr<GeometricFrame> frame1,
	std::shared_ptr<GeometricFrame> frame2
)
{
	KDL::Vector ax1 = pose_a_.M * frame1->getAxisXKDL();
	KDL::Vector ax2 = pose_b_.M * frame2->getAxisXKDL();
	KDL::Vector ay1 = pose_a_.M * frame1->getAxisYKDL();
	KDL::Vector ay2 = pose_b_.M * frame2->getAxisYKDL();

	double d1 = KDL::dot(ax1, ax2);
	double d2 = KDL::dot(ay1, ay2);

	e_(0) = d1 - std::cos(delta_);
	e_(1) = d2 - std::cos(delta_);

  KDL::Vector v1 = ax1 * ax2;
  KDL::Vector v2 = ay1 * ay2;

  for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr) {
  	KDL::Vector Ja = jacobian_a_.getColumn(q_nr).rot;
    KDL::Vector Jb = jacobian_b_.getColumn(q_nr).rot;
    J_(0, q_nr) = KDL::dot( v1, (Ja - Jb) );
    J_(1, q_nr) = KDL::dot( v2, (Ja - Jb) );
  }

  return 0;
}





} // namespace tasks

} // namespace hiqp












