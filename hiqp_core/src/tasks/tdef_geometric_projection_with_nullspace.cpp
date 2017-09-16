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

  KDL::Vector x2 = x;
  x2(2) = 0;
  KDL::Vector n = x2*v;

    KDL::Vector m = x;
    m(0) = 0;
    m(1) = 0;
    m(2) = 1;
  n.Normalize();
  v.Normalize();
  for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr) {
    KDL::Vector Jpd = -getVelocityJacobianForTwoPoints(p__, d__, q_nr);
    // KDL::Vector Jpn = -getVelocityJacobianForOnePoint(p__, q_nr);
    KDL::Vector y = K * Jpd;
    J_(0, q_nr) = 2 * KDL::dot(x, y);
    J_(1, q_nr) = KDL::dot(n, Jpd);
    J_(2, q_nr) = KDL::dot(m, Jpd);
  }
  return 0;
}

template <>
int TDefGeometricProjectionWithNullspace<GeometricPoint, GeometricPlane>::project(
    std::shared_ptr<GeometricPoint> point,
    std::shared_ptr<GeometricPlane> plane) {
  KDL::Vector p__ = pose_a_.M * point->getPointKDL();
  KDL::Vector p = pose_a_.p + p__;

  KDL::Vector n = pose_b_.M * plane->getNormalKDL();

  KDL::Vector d__ = n * plane->getOffset();
  KDL::Vector d = d__ + n * KDL::dot(n, pose_b_.p);

  e_(0) = KDL::dot(n, (p - d));

  KDL::Vector v1(1,0,0);
  KDL::Vector v2(0,1,0);

  for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr) {
    KDL::Vector Jpd = -getVelocityJacobianForTwoPoints(p__, d__, q_nr);
    J_(0, q_nr) = KDL::dot(n, Jpd);
    J_(1, q_nr) = KDL::dot(v1, Jpd);
    J_(2, q_nr) = KDL::dot(v2, Jpd);

  }

  return 0;
}

///////////////////////////////////////////////////////////////////////////////
//  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
///////////////////////////////////////////////////////////////////////////////
//
//                                 S P H E R E
//
///////////////////////////////////////////////////////////////////////////////

// template <>
// int TDefGeometricProjection<GeometricPoint, GeometricSphere>::project(
//     std::shared_ptr<GeometricPoint> point,
//     std::shared_ptr<GeometricSphere> sphere) {
//   KDL::Vector p1__ = pose_a_.M * point->getPointKDL();
//   KDL::Vector p1 = pose_a_.p + p1__;

//   KDL::Vector p2__ = pose_b_.M * sphere->getCenterKDL();
//   KDL::Vector p2 = pose_b_.p + p2__;

//   KDL::Vector d = p2 - p1;
//   e_(0) = KDL::dot(d, d) - sphere->getRadius() * sphere->getRadius();

//   KDL::Vector x(1,0,0);
//   KDL::Vector z(0,0,1);

//   KDL::Vector xn = d*x;
//   KDL::Vector zn = d*z;

//   xn.Normalize();
//   zn.Normalize();

//   // The task jacobian is J = 2 (p2-p1)^T (Jp2 - Jp1)
//   for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr) {
//     KDL::Vector Jp2p1 = getVelocityJacobianForTwoPoints(p1__, p2__, q_nr);
//     J_(0, q_nr) = 2 * dot(d, Jp2p1);
//   }
//   return 0;
// }

}  // namespace tasks

}  // namespace hiqp
