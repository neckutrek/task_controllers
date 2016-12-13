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

#ifndef HIQP_GEOMETRIC_PRIMITIVE_COUTER_H
#define HIQP_GEOMETRIC_PRIMITIVE_COUTER_H

#include <memory>

#include <hiqp/geometric_primitives/geometric_primitive_visitor.h>
#include <hiqp/visualizer.h>

namespace hiqp {

namespace geometric_primitives {

  /*! \brief A Geometric Primitive Visitor that prints info about the primitive it visits to std::cout.
   *  \author Marcus A Johansson */
  class GeometricPrimitiveCouter : public GeometricPrimitiveVisitor {
  public:
    GeometricPrimitiveCouter() = default;

    ~GeometricPrimitiveCouter() noexcept = default;

    void visit(std::shared_ptr<GeometricPoint> point) { print(point); std::cout << "point\n"; }
    void visit(std::shared_ptr<GeometricLine> line) { print(line); std::cout << "line\n"; }
    void visit(std::shared_ptr<GeometricPlane> plane) { print(plane); std::cout << "plane\n"; }
    void visit(std::shared_ptr<GeometricBox> box) { print(box); std::cout << "box\n"; }
    void visit(std::shared_ptr<GeometricCylinder> cylinder) { print(cylinder); std::cout << "cylinder\n"; }
    void visit(std::shared_ptr<GeometricSphere> sphere) { print(sphere); std::cout << "sphere\n"; }
    void visit(std::shared_ptr<GeometricFrame> frame) { print(frame); std::cout << "frame\n"; }


  private:
    GeometricPrimitiveCouter(const GeometricPrimitiveCouter& other) = delete;
    GeometricPrimitiveCouter(GeometricPrimitiveCouter&& other) = delete;
    GeometricPrimitiveCouter& operator=(const GeometricPrimitiveCouter& other) = delete;
    GeometricPrimitiveCouter& operator=(GeometricPrimitiveCouter&& other) noexcept = delete;

    template <typename Primitive>
    void print(std::shared_ptr<Primitive> primitive);

  };

  template <typename Primitive>
  void GeometricPrimitiveCouter::print(std::shared_ptr<Primitive> primitive) {
    std::cout << primitive->getName() << ", " 
              << primitive->getFrameId() << ", "
              << primitive->isVisible() << ", "
              << primitive->getVisualId() << ", ";
  }

} // namespace geometric_primitives

} // namespace hiqp

#endif // include guard