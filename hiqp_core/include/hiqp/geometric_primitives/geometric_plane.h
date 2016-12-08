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

#ifndef HIQP_GEOMETRIC_PLANE_H
#define HIQP_GEOMETRIC_PLANE_H

#include <hiqp/geometric_primitives/geometric_primitive.h>
#include <hiqp/utilities.h>

#include <kdl/frames.hpp>

namespace hiqp
{
namespace geometric_primitives
{

  /*! \brief Parameters: [n.x, n.y, n.z, offset]. The plane equation corresponds to \f$n^T*x-offset=0\f$.
   *  \author Marcus A Johansson */ 
  class GeometricPlane : public GeometricPrimitive {
  public:
    GeometricPlane(const std::string& name,
                  const std::string& type,
                   const std::string& frame_id,
                   bool visible,
                   const std::vector<double>& color)
      : GeometricPrimitive(name, type, frame_id, visible, color) {}

    ~GeometricPlane() noexcept = default;

    /*! \brief Parses a set of parameters and initializes the plane.
     *  \param parameters : Should be of size 4.<ol>
     *                      <li>Indices 0-2 (required) defines the normal vector of the plane.</li>
     *                      <li>Index 3 (required) defines the offset of the plane in the normal direction.</li>
     *                      </ol>
     * \return 0 on success, -1 if the wrong number of parameters was sent. */
    int init(const std::vector<double>& parameters) {
      int size = parameters.size();
      if (size != 4) {
        printHiqpWarning("GeometricPlane requires 4 parameters, got " 
          + std::to_string(size) + "! Initialization failed!");
        return -1;
      }

      kdl_n_(0) = parameters.at(0);
      kdl_n_(1) = parameters.at(1);
      kdl_n_(2) = parameters.at(2);

      kdl_n_.Normalize();
      eigen_n_ << kdl_n_(0), kdl_n_(1), kdl_n_(2);
      d_ = parameters.at(3);
      return 0;
    }

    inline const KDL::Vector&      getNormalKDL()    { return kdl_n_; }
    inline const Eigen::Vector3d&  getNormalEigen()  { return eigen_n_; }

    inline double getOffset() { return d_; }
    inline double getNormalX() { return kdl_n_(0); }
    inline double getNormalY() { return kdl_n_(1); }
    inline double getNormalZ() { return kdl_n_(2); }

  protected:
    KDL::Vector      kdl_n_; // the normal vector or the plane
    Eigen::Vector3d  eigen_n_;
    double           d_; // the offset in the normal direction

  private:
    GeometricPlane(const GeometricPlane& other) = delete;
    GeometricPlane(GeometricPlane&& other) = delete;
    GeometricPlane& operator=(const GeometricPlane& other) = delete;
    GeometricPlane& operator=(GeometricPlane&& other) noexcept = delete;
  };

} // namespace geometric_primitives

} // namespace hiqp

#endif // include guard
