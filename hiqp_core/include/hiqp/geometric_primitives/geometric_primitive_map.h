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

#ifndef HIQP_GEOMETRIC_PRIMITIVE_MAP_H
#define HIQP_GEOMETRIC_PRIMITIVE_MAP_H

#include <map>
#include <memory>

#include <hiqp/geometric_primitives/geometric_point.h>
#include <hiqp/geometric_primitives/geometric_line.h>
#include <hiqp/geometric_primitives/geometric_plane.h>
#include <hiqp/geometric_primitives/geometric_box.h>
#include <hiqp/geometric_primitives/geometric_cylinder.h>
#include <hiqp/geometric_primitives/geometric_sphere.h>
#include <hiqp/geometric_primitives/geometric_frame.h>
#include <hiqp/geometric_primitives/geometric_primitive_visitor.h>

namespace hiqp
{
namespace geometric_primitives
{

  /*! \brief A common map data structure for all geometric primitive types.
   *  \author Marcus A Johansson */  
  class GeometricPrimitiveMap {
  public:
    GeometricPrimitiveMap() {}
    ~GeometricPrimitiveMap() noexcept {}

    int addGeometricPrimitive(const std::string& name,
                              const std::string& type,
                              const std::string& frame_id,
                              bool visible,
                              const std::vector<double>& color,
                              const std::vector<double>& parameters);

    int removeGeometricPrimitive(std::string name);

    int clear();

    template<typename PrimitiveType>
    std::shared_ptr<PrimitiveType> getGeometricPrimitive(const std::string& name);

    template<typename PrimitiveType>
    void updateGeometricPrimitive(const std::string& name, 
                                  const std::vector<double>& parameters);

    void addDependencyToPrimitive(const std::string& name, const std::string& id);
    void removeDependency(const std::string& id);
    void acceptVisitor(GeometricPrimitiveVisitor& visitor, const std::string& primitive_name = "");

  private:
    GeometricPrimitiveMap(const GeometricPrimitiveMap& other) = delete;
    GeometricPrimitiveMap(GeometricPrimitiveMap&& other) = delete;
    GeometricPrimitiveMap& operator=(const GeometricPrimitiveMap& other) = delete;
    GeometricPrimitiveMap& operator=(GeometricPrimitiveMap&& other) noexcept = delete;

    std::string getDependenciesAsString(std::string name);

    typedef std::map< std::string, std::vector< std::string> >          DependencyMap;
    typedef std::map< std::string, std::shared_ptr<GeometricPoint> >    PointMap;
    typedef std::map< std::string, std::shared_ptr<GeometricLine> >     LineMap;
    typedef std::map< std::string, std::shared_ptr<GeometricPlane> >    PlaneMap;
    typedef std::map< std::string, std::shared_ptr<GeometricBox> >      BoxMap;
    typedef std::map< std::string, std::shared_ptr<GeometricCylinder> > CylinderMap;
    typedef std::map< std::string, std::shared_ptr<GeometricSphere> >   SphereMap;
    typedef std::map< std::string, std::shared_ptr<GeometricFrame> >    FrameMap;

    DependencyMap   dependency_map_;
    PointMap        point_map_;
    LineMap         line_map_;
    PlaneMap        plane_map_;
    BoxMap          box_map_;
    CylinderMap     cylinder_map_;
    SphereMap       sphere_map_;
    FrameMap        frame_map_;

    std::vector<std::string> all_primitive_names_;

  };

} // namespace geometric_primitives

} // namespace hiqp

#endif // include guard