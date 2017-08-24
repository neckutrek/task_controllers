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

#ifndef HIQP_GEOMETRIC_FRAME_H
#define HIQP_GEOMETRIC_FRAME_H

#include <hiqp/geometric_primitives/geometric_primitive.h>
#include <hiqp/utilities.h>

#include <kdl/frames.hpp>

#include <Eigen/Dense>

namespace hiqp {
  namespace geometric_primitives {

/*! \brief Parameters: [x, y, z]
 *                     [x, y, z, ax, ay, az]
 *                     [x, y, z, qw, qx, qy, qz]
 *  \author Marcus A Johansson */
    class GeometricFrame : public GeometricPrimitive {
    public:
      GeometricFrame(const std::string& name, const std::string& frame_id,
       bool visible, const std::vector<double>& color)
      : GeometricPrimitive(name, frame_id, visible, color) {}

      ~GeometricFrame() noexcept {}

      int init(const std::vector<double>& parameters) {
        GeometricPrimitive::init(parameters);
        int size = parameters.size();
        if (!(size == 3 || size == 6 || size == 7)) {
          printHiqpWarning("GeometricFrame requires 3, 6 or 7 parameters, got " +
           std::to_string(size) + "! Initialization failed!");
          return -1;
        }

        kdl_c_(0) = parameters.at(0);
        kdl_c_(1) = parameters.at(1);
        kdl_c_(2) = parameters.at(2);

        eigen_c_ << kdl_c_(0), kdl_c_(1), kdl_c_(2);

        if (size == 6) {
          double angle1 = parameters.at(3);
          double angle2 = parameters.at(4);
          double angle3 = parameters.at(5);

          Eigen::Matrix3d m;
          m = Eigen::AngleAxisd(angle1, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(angle2, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(angle3, Eigen::Vector3d::UnitZ());
          q_ = Eigen::Quaternion<double>(m);

        } else if (size == 7) {
          double w = parameters.at(3);
          double x = parameters.at(4);
          double y = parameters.at(5);
          double z = parameters.at(6);
          q_ = Eigen::Quaternion<double>(w, x, y, z);

        } else {
          q_ = Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0);
        }

        Eigen::Vector3d ax = q_._transformVector(Eigen::Vector3d(1, 0, 0));
        axis_x_ = KDL::Vector(ax(0), ax(1), ax(2));

        Eigen::Vector3d ay = q_._transformVector(Eigen::Vector3d(0, 1, 0));
        axis_y_ = KDL::Vector(ay(0), ay(1), ay(2));

        Eigen::Vector3d az = q_._transformVector(Eigen::Vector3d(0, 0, 1));
        axis_z_ = KDL::Vector(az(0), az(1), az(2));

        return 0;
      }

      inline const KDL::Vector& getCenterKDL() { return kdl_c_; }
      inline const Eigen::Vector3d& getCenterEigen() { return eigen_c_; }
      inline const Eigen::Quaternion<double>& getQuaternionEigen() { return q_; }

      inline const void setCenterOffsetKDL(KDL::Vector& kdl_c_new) {
       kdl_c_=kdl_c_new;
       eigen_c_ << kdl_c_(0), kdl_c_(1), kdl_c_(2); 
     }
     inline const void addAngleOffsetQuaternionEigen(Eigen::Quaternion<double>& q_new) {
      q_ = q_new;
      Eigen::Vector3d ax = q_._transformVector(Eigen::Vector3d(1, 0, 0));
      axis_x_ = KDL::Vector(ax(0), ax(1), ax(2));

      Eigen::Vector3d ay = q_._transformVector(Eigen::Vector3d(0, 1, 0));
      axis_y_ = KDL::Vector(ay(0), ay(1), ay(2));

      Eigen::Vector3d az = q_._transformVector(Eigen::Vector3d(0, 0, 1));
      axis_z_ = KDL::Vector(az(0), az(1), az(2));
//Eigen::Quaternion<double>(q_new.w(), q_.x()+q_new.x(), q_.y()+q_new.y(), q_.z()+q_new.z());
    }

    inline const KDL::Vector& getAxisXKDL() { return axis_x_; }
    inline const KDL::Vector& getAxisYKDL() { return axis_y_; }
    inline const KDL::Vector& getAxisZKDL() { return axis_z_; }

    inline double getX() { return kdl_c_(0); }
    inline double getY() { return kdl_c_(1); }
    inline double getZ() { return kdl_c_(2); }

    inline double getQW() { return q_.w(); }
    inline double getQX() { return q_.x(); }
    inline double getQY() { return q_.y(); }
    inline double getQZ() { return q_.z(); }

  protected:
    KDL::Vector kdl_c_;
    Eigen::Vector3d eigen_c_;

    KDL::Vector axis_x_;
    KDL::Vector axis_y_;
    KDL::Vector axis_z_;

    Eigen::Quaternion<double> q_;

  private:
    GeometricFrame(const GeometricFrame& other) = delete;
    GeometricFrame(GeometricFrame&& other) = delete;
    GeometricFrame& operator=(const GeometricFrame& other) = delete;
    GeometricFrame& operator=(GeometricFrame&& other) noexcept = delete;

};  // class Geometric Point

}  // namespace geometric_primitives

}  // namespace hiqp

#endif  // include guard
