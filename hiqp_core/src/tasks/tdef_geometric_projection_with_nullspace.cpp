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

#include <hiqp/tasks/tdef_geometric_projection_with_nullspace.h>

#include <hiqp/tasks/tdef_geometric_projection.h>
#include <hiqp/geometric_primitives/geometric_box.h>
#include <hiqp/geometric_primitives/geometric_cylinder.h>
#include <hiqp/geometric_primitives/geometric_frame.h>
#include <hiqp/geometric_primitives/geometric_line.h>
#include <hiqp/geometric_primitives/geometric_plane.h>
#include <hiqp/geometric_primitives/geometric_point.h>
#include <hiqp/geometric_primitives/geometric_sphere.h>

#include <hiqp/utilities.h>

#include <iostream>
#include <sstream>
#include <string>

namespace hiqp {
namespace tasks {

/// \todo Implement point-capsule projection
/// \todo Implement cylinder-cylinder projection
/// \todo Implement cylinder-sphere projection
/// \todo Implement cylinder-capsule projection
/// \todo Implement sphere-capsule projection
/// \todo Implement capsule-capsule projection
/// \todo Implement activation zones for all tasks

///////////////////////////////////////////////////////////////////////////////
//  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
///////////////////////////////////////////////////////////////////////////////
//
//                                 P O I N T
//
///////////////////////////////////////////////////////////////////////////////


template <>
int TDefGeometricProjectionWithNullspace<GeometricPoint, GeometricCylinder>::project(
    std::shared_ptr<GeometricPoint> point,
    std::shared_ptr<GeometricCylinder> cylinder) {
  KDL::Vector p__ = pose_a_.M * point->getPointKDL();
  KDL::Vector p = pose_a_.p + p__;

  KDL::Vector v = pose_b_.M * cylinder->getDirectionKDL();

  KDL::Vector d__ = pose_b_.M * cylinder->getOffsetKDL();
  KDL::Vector d = pose_b_.p + d__;

  KDL::Vector x = p - d;
  double s = KDL::dot(x, v);

  e_(0) =
      KDL::dot(x, x) - s * s - cylinder->getRadius() * cylinder->getRadius();

  // The task jacobian is J = 2 (p-d)^T (I-vv^T) (Jp-Jd)

  // As KDL does not provide a KDL::Matrix class, we use KDL::Rotation
  // although K is not an actual rotation matrix in this context !
  // K = (I - v v^T)
  KDL::Rotation K = KDL::Rotation(KDL::Vector(1, 0, 0) - v * v(0),
                                  KDL::Vector(0, 1, 0) - v * v(1),
                                  KDL::Vector(0, 0, 1) - v * v(2));

  KDL::Vector n = v*x;

  for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr) {
    KDL::Vector Jpd = -getVelocityJacobianForTwoPoints(p__, d__, q_nr);
    KDL::Vector Jpn = getVelocityJacobianForOnePoint(p__, q_nr);
    KDL::Vector y = K * Jpd;
    J_(0, q_nr) = 2 * KDL::dot(x, y);
    J_(1, q_nr) = KDL::dot(n, Jpn);
  }
  return 0;
}


///////////////////////////////////////////////////////////////////////////////
//  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
///////////////////////////////////////////////////////////////////////////////
//
//                                 L I N E
//
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
//  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
///////////////////////////////////////////////////////////////////////////////
//
//                                 S P H E R E
//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
///////////////////////////////////////////////////////////////////////////////
//
//                                 F R A M E
//
///////////////////////////////////////////////////////////////////////////////

}  // namespace tasks

}  // namespace hiqp
