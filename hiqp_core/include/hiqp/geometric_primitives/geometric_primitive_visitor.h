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

#ifndef HIQP_GEOMETRIC_PRIMITIVE_VISITOR_H
#define HIQP_GEOMETRIC_PRIMITIVE_VISITOR_H

#include <memory>

#include <hiqp/geometric_primitives/geometric_point.h>
#include <hiqp/geometric_primitives/geometric_line.h>
#include <hiqp/geometric_primitives/geometric_plane.h>
#include <hiqp/geometric_primitives/geometric_box.h>
#include <hiqp/geometric_primitives/geometric_cylinder.h>
#include <hiqp/geometric_primitives/geometric_sphere.h>
#include <hiqp/geometric_primitives/geometric_frame.h>

namespace hiqp {

namespace geometric_primitives {

  /*! \brief 
   * {author Marcus A Johansson */
  class GeometricPrimitiveVisitor {
  public:
    GeometricPrimitiveVisitor() {}
    ~GeometricPrimitiveVisitor() noexcept {}

    virtual void visit(std::shared_ptr<GeometricPoint> point) = 0;
    virtual void visit(std::shared_ptr<GeometricLine> line) = 0;
    virtual void visit(std::shared_ptr<GeometricPlane> plane) = 0;
    virtual void visit(std::shared_ptr<GeometricBox> box) = 0;
    virtual void visit(std::shared_ptr<GeometricCylinder> cylinder) = 0;
    virtual void visit(std::shared_ptr<GeometricSphere> sphere) = 0;
    virtual void visit(std::shared_ptr<GeometricFrame> frame) = 0;

  private:
    GeometricPrimitiveVisitor(const GeometricPrimitiveVisitor& other) = delete;
    GeometricPrimitiveVisitor(GeometricPrimitiveVisitor&& other) = delete;
    GeometricPrimitiveVisitor& operator=(const GeometricPrimitiveVisitor& other) = delete;
    GeometricPrimitiveVisitor& operator=(GeometricPrimitiveVisitor&& other) noexcept = delete;
  };

} // namespace geometric_primitives

} // namespace hiqp

#endif // include guard