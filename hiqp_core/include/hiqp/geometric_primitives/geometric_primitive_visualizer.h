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

#ifndef HIQP_GEOMETRIC_PRIMITIVE_VISUALIZER_H
#define HIQP_GEOMETRIC_PRIMITIVE_VISUALIZER_H

#include <memory>

#include <hiqp/geometric_primitives/geometric_primitive_visitor.h>
#include <hiqp/visualizer.h>

namespace hiqp {

namespace geometric_primitives {

  /*! \brief A Geometric Primitive Visitor that sends the primitive it visits to a visualizer.
   *  \author Marcus A Johansson */
  class GeometricPrimitiveVisualizer : public GeometricPrimitiveVisitor {
  public:
    /*! \brief Adds to, updates or removes a primitive from the visualizer given. If action=1, removeAllVisitedPrimitives has to be called after finishing visiting to perform the actual removal.
     *  \param action : 0 for adding/updating the visited geometric primitive to the visualizer, 1 for removing it. */
    GeometricPrimitiveVisualizer(std::shared_ptr<Visualizer> visualizer, int action)
    : visualizer_(visualizer), action_(action)
    {}

    ~GeometricPrimitiveVisualizer() noexcept {}

    void visit(std::shared_ptr<GeometricPoint> point) { visit__(point); }
    void visit(std::shared_ptr<GeometricLine> line) { visit__(line); }
    void visit(std::shared_ptr<GeometricPlane> plane) { visit__(plane); }
    void visit(std::shared_ptr<GeometricBox> box) { visit__(box); }
    void visit(std::shared_ptr<GeometricCylinder> cylinder) { visit__(cylinder); }
    void visit(std::shared_ptr<GeometricSphere> sphere) { visit__(sphere); }
    void visit(std::shared_ptr<GeometricFrame> frame);

    /// \brief Effectively removes all visited geometric primitives.
    void removeAllVisitedPrimitives() {
      visualizer_->removeMany(visited_visual_ids_);
    }

  private:
    GeometricPrimitiveVisualizer(const GeometricPrimitiveVisualizer& other) = delete;
    GeometricPrimitiveVisualizer(GeometricPrimitiveVisualizer&& other) = delete;
    GeometricPrimitiveVisualizer& operator=(const GeometricPrimitiveVisualizer& other) = delete;
    GeometricPrimitiveVisualizer& operator=(GeometricPrimitiveVisualizer&& other) noexcept = delete;

    template <typename Primitive>
    void visit__(std::shared_ptr<Primitive> primitive);

    std::shared_ptr<Visualizer>  visualizer_;
    int                          action_;
    std::vector<int>             visited_visual_ids_;
  };

  template <typename Primitive>
  void GeometricPrimitiveVisualizer::visit__(std::shared_ptr<Primitive> primitive) {
    int id = primitive->getVisualId();
    switch (action_) {
    case 0:
      if (id < 0) primitive->setVisualId(visualizer_->add(primitive));
      else        visualizer_->update(id, primitive);
      break;
    case 1:
      if (id >= 0) visited_visual_ids_.push_back(id);
      break;
    default:
      break;
    }
  }

  template <>
  void GeometricPrimitiveVisualizer::visit__<GeometricFrame>(std::shared_ptr<GeometricFrame> primitive) {
    int id = primitive->getVisualId();
    switch (action_) {
    case 0:
      if (id < 0) primitive->setVisualId(visualizer_->add(primitive));
      else        visualizer_->update(id, primitive);
      break;
    case 1:
      if (id >= 0) {
        // the frame primitive consists of three successive visual markers
        visited_visual_ids_.push_back(id);
        visited_visual_ids_.push_back(id+1);
        visited_visual_ids_.push_back(id+2);
      }
      break;
    default:
      break;
    }
  }

  void GeometricPrimitiveVisualizer::visit(std::shared_ptr<GeometricFrame> frame) {
    visit__(frame);
  }

} // namespace geometric_primitives

} // namespace hiqp

#endif // include guard