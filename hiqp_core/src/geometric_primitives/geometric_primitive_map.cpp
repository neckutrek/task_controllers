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

#include <iostream>
#include <algorithm>
#include <iterator>

#include <hiqp/geometric_primitives/geometric_primitive_map.h>
#include <hiqp/utilities.h>

namespace hiqp
{
namespace geometric_primitives
{

/// \todo Add a Capsule primitive among the geometric primitives
int GeometricPrimitiveMap::addGeometricPrimitive
(
  const std::string& name,
  const std::string& type,
  const std::string& frame_id,
  bool visible,
  const std::vector<double>& color,
  const std::vector<double>& parameters
)
{
  if (std::find(all_primitive_names_.begin(), all_primitive_names_.end(), name)
      != all_primitive_names_.end())
  {
    printHiqpWarning("A primitive with name '" + name 
      + "' already exists. No new primitive was added!");
    return -1;
  }

  bool success = false;

  if (type.compare("point") == 0)
  {
    auto primitive = std::make_shared<GeometricPoint>(name, type, frame_id, visible, color);
    if (primitive->init(parameters) == 0)
    {
      point_map_.emplace( name, primitive );
      success = true;
    }
  }
  else if (type.compare("line") == 0)
  {
    auto primitive = std::make_shared<GeometricLine>(name, type, frame_id, visible, color);
    if (primitive->init(parameters) == 0)
    {
      line_map_.emplace( name, primitive );
      success = true;
    }

  }
  else if (type.compare("plane") == 0)
  {
    auto primitive = std::make_shared<GeometricPlane>(name, type, frame_id, visible, color);
    if (primitive->init(parameters) == 0)
    {
      plane_map_.emplace( name, primitive );
      success = true;
    }

  }
  else if (type.compare("box") == 0)
  {
    auto primitive = std::make_shared<GeometricBox>(name, type, frame_id, visible, color);
    if (primitive->init(parameters) == 0)
    {
      box_map_.emplace( name, primitive );
      success = true;
    }
  }
  else if (type.compare("cylinder") == 0)
  {
    auto primitive = std::make_shared<GeometricCylinder>(name, type, frame_id, visible, color);
    if (primitive->init(parameters) == 0)
    {
      cylinder_map_.emplace( name, primitive );
      success = true;
    }
  }
  else if (type.compare("sphere") == 0)
  {
    auto primitive = std::make_shared<GeometricSphere>(name, type,  frame_id, visible, color);
    if (primitive->init(parameters) == 0)
    {
      sphere_map_.emplace( name, primitive );
      success = true;
    }
  }
  else if (type.compare("frame") == 0)
  {
    auto primitive = std::make_shared<GeometricFrame>(name, type, frame_id, visible, color);
    if (primitive->init(parameters) == 0)
    {
      frame_map_.emplace( name, primitive );
      success = true;
    }
  }
  else
  {
    printHiqpWarning("Couldn't parse geometric type '" + type + 
      "'. No new primitive was added!");
    return -2;
  }

  if (success)
  {
    all_primitive_names_.push_back(name);
    dependency_map_.emplace( name, std::vector<std::string>() );
  } 
  else
  {
    return -3;
  }

  return 0;
}





int GeometricPrimitiveMap::removeGeometricPrimitive(std::string name) {
  if (std::find(all_primitive_names_.begin(), all_primitive_names_.end(), name)
      == all_primitive_names_.end())
  {
    printHiqpWarning("While trying to remove primitive with name '" + name 
      + "', could not find that primitive. No primitive was removed!");
    return -1;
  }

  std::string dependencies = getDependenciesAsString(name);
  if (dependencies.size() > 0)
  {
    printHiqpWarning("Geometric primitive '" + name + 
        "' has the following dependencies: " + dependencies + 
        " and could not be deleted. Remove the dependencies first!");
      return -2;
  }

  point_map_.erase(name);
  line_map_.erase(name);
  plane_map_.erase(name);
  box_map_.erase(name);
  cylinder_map_.erase(name);
  sphere_map_.erase(name);
  frame_map_.erase(name);
  dependency_map_.erase(name);
  
  all_primitive_names_.erase(
    std::remove(all_primitive_names_.begin(), all_primitive_names_.end(), name), 
    all_primitive_names_.end()
  );

  printHiqpInfo("Removed geometric primitive '" + name + "'.");

  return 0;
}



int GeometricPrimitiveMap::clear
()
{
  std::vector<std::string> names_to_remove;

  for (auto&& name : all_primitive_names_)
  {
    std::string dependencies = getDependenciesAsString(name);
    if (dependencies.size() > 0)
    {
      printHiqpWarning("Geometric primitive '" + name + 
          "' has the following dependencies: " + dependencies + 
          " and could not be deleted. Remove the dependencies first!");
    }
    else
    {
      names_to_remove.push_back(name);
    }
  }

  for (auto&& name : names_to_remove)
  {
    point_map_.erase(name);
    line_map_.erase(name);
    plane_map_.erase(name);
    box_map_.erase(name);
    cylinder_map_.erase(name);
    sphere_map_.erase(name);
    frame_map_.erase(name);
    dependency_map_.erase(name);

    all_primitive_names_.erase(
      std::remove(all_primitive_names_.begin(), all_primitive_names_.end(), name), 
      all_primitive_names_.end()
    );
  }

  return 0;
}





void GeometricPrimitiveMap::addDependencyToPrimitive
(
  const std::string& primitive_name, 
  const std::string& dependency_name
)
{
  DependencyMap::iterator it = dependency_map_.find(primitive_name);

  if (it == dependency_map_.end())
  {
    printHiqpWarning("Trying to add dependency to geometric primitive '" 
      + primitive_name + "'. No such primitive found. No dependency was added!");
  }

  std::vector<std::string>::iterator it2 = std::find(
    it->second.begin(), it->second.end(), dependency_name
  );

  if (it2 == it->second.end())
    it->second.push_back(dependency_name);
}





void GeometricPrimitiveMap::removeDependency
(
  const std::string& dependency_name
)
{
  for (auto&& kv : dependency_map_)
  {
    kv.second.erase(std::remove(kv.second.begin(), kv.second.end(), dependency_name),
                    kv.second.end());
  }

  // DependencyMap::iterator it = dependency_map_.begin();
  // while (it != dependency_map_.end())
  // {
  //   it->second.erase(
  //     std::remove(
  //       it->second.begin(), 
  //       it->second.end(), 
  //       dependency_name)
  //     , 
  //     it->second.end()
  //     );

  //   ++it;
  // }
}





void GeometricPrimitiveMap::acceptVisitor(GeometricPrimitiveVisitor& visitor, 
                                          const std::string& primitive_name) {
  if (primitive_name.compare("") == 0) {
    for (auto&& kv : point_map_) visitor.visit(kv.second);
    for (auto&& kv : line_map_) visitor.visit(kv.second);
    for (auto&& kv : plane_map_) visitor.visit(kv.second);
    for (auto&& kv : box_map_) visitor.visit(kv.second);
    for (auto&& kv : cylinder_map_) visitor.visit(kv.second);
    for (auto&& kv : sphere_map_) visitor.visit(kv.second);
    for (auto&& kv : frame_map_) visitor.visit(kv.second);
  } else {
    if (std::find(all_primitive_names_.begin(), all_primitive_names_.end(), primitive_name) 
          != all_primitive_names_.end()) {
      {
        PointMap::iterator it = point_map_.find(primitive_name);
        if (it != point_map_.end()) visitor.visit(it->second);
      }
      {
        LineMap::iterator it = line_map_.find(primitive_name);
        if (it != line_map_.end()) visitor.visit(it->second);
      }
      {
        PlaneMap::iterator it = plane_map_.find(primitive_name);
        if (it != plane_map_.end()) visitor.visit(it->second);
      }
      {
        BoxMap::iterator it = box_map_.find(primitive_name);
        if (it != box_map_.end()) visitor.visit(it->second);
      }
      {
        CylinderMap::iterator it = cylinder_map_.find(primitive_name);
        if (it != cylinder_map_.end()) visitor.visit(it->second);
      }
      {
        SphereMap::iterator it = sphere_map_.find(primitive_name);
        if (it != sphere_map_.end()) visitor.visit(it->second);
      }
      {
        FrameMap::iterator it = frame_map_.find(primitive_name);
        if (it != frame_map_.end()) visitor.visit(it->second);
      }
    }
  }
}




////////////////////////////////////////////////////////////////////////////////
//  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
////////////////////////////////////////////////////////////////////////////////
//
//              G E T   G E O M E T R I C   P R I M I T I V E
//
////////////////////////////////////////////////////////////////////////////////

template<>
std::shared_ptr<GeometricPoint>
GeometricPrimitiveMap::getGeometricPrimitive<GeometricPoint>
(
  const std::string& name
)
{
  PointMap::iterator it = point_map_.find(name);
  if (it  == point_map_.end())   return nullptr;
  else                           return it->second;
}





template<>
std::shared_ptr<GeometricLine>
GeometricPrimitiveMap::getGeometricPrimitive<GeometricLine>
(
  const std::string& name
)
{
  LineMap::iterator it = line_map_.find(name);
  if (it  == line_map_.end())   return nullptr;
  else                          return it->second;
}





template<>
std::shared_ptr<GeometricPlane>
GeometricPrimitiveMap::getGeometricPrimitive<GeometricPlane>
(
  const std::string& name
)
{
  PlaneMap::iterator it = plane_map_.find(name);
  if (it  == plane_map_.end())   return nullptr;
  else                           return it->second;
}





template<>
std::shared_ptr<GeometricBox>
GeometricPrimitiveMap::getGeometricPrimitive<GeometricBox>
(
  const std::string& name
)
{
  BoxMap::iterator it = box_map_.find(name);
  if (it  == box_map_.end())     return nullptr;
  else                           return it->second;
}





template<>
std::shared_ptr<GeometricCylinder>
GeometricPrimitiveMap::getGeometricPrimitive<GeometricCylinder>
(
  const std::string& name
)
{
  CylinderMap::iterator it = cylinder_map_.find(name);
  if (it  == cylinder_map_.end())     return nullptr;
  else                                return it->second;
}





template<>
std::shared_ptr<GeometricSphere>
GeometricPrimitiveMap::getGeometricPrimitive<GeometricSphere>
(
  const std::string& name
)
{
  SphereMap::iterator it = sphere_map_.find(name);
  if (it  == sphere_map_.end())       return nullptr;
  else                                return it->second;
}





template<>
std::shared_ptr<GeometricFrame>
GeometricPrimitiveMap::getGeometricPrimitive<GeometricFrame>
(
  const std::string& name
)
{
  FrameMap::iterator it = frame_map_.find(name);
  if (it  == frame_map_.end())        return nullptr;
  else                                return it->second;
}





////////////////////////////////////////////////////////////////////////////////
//  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
////////////////////////////////////////////////////////////////////////////////
//
//             U P D A T E   G E O M E T R I C   P R I M I T I V E 
//
////////////////////////////////////////////////////////////////////////////////

template<>
void GeometricPrimitiveMap::updateGeometricPrimitive<GeometricPoint>
(
  const std::string& name, 
  const std::vector<double>& parameters
)
{
  PointMap::iterator it = point_map_.find(name);
  if (it == point_map_.end()) 
  {
    printHiqpWarning("Couldn't update GeometricPoint with name '" + 
      name + "'. The point was not found!");
    return;
  }
  it->second->init(parameters);
}





template<>
void GeometricPrimitiveMap::updateGeometricPrimitive<GeometricLine>
(
  const std::string& name, 
  const std::vector<double>& parameters
)
{
  LineMap::iterator it = line_map_.find(name);
  if (it == line_map_.end()) 
  {
    printHiqpWarning("Couldn't update GeometricLine with name '" + 
      name + "'. The line was not found!");
    return;
  }
  it->second->init(parameters);
}





template<>
void GeometricPrimitiveMap::updateGeometricPrimitive<GeometricPlane>
(
  const std::string& name, 
  const std::vector<double>& parameters
)
{
  PlaneMap::iterator it = plane_map_.find(name);
  if (it == plane_map_.end()) 
  {
    printHiqpWarning("Couldn't update GeometricPlane with name '" + 
      name + "'. The plane was not found!");
    return;
  }
  it->second->init(parameters);
}





template<>
void GeometricPrimitiveMap::updateGeometricPrimitive<GeometricBox>
(
  const std::string& name, 
  const std::vector<double>& parameters
)
{
  BoxMap::iterator it = box_map_.find(name);
  if (it == box_map_.end()) 
  {
    printHiqpWarning("Couldn't update GeometricBox with name '" + 
      name + "'. The box was not found!");
    return;
  }
  it->second->init(parameters);
}





template<>
void GeometricPrimitiveMap::updateGeometricPrimitive<GeometricCylinder>
(
  const std::string& name, 
  const std::vector<double>& parameters
)
{
  CylinderMap::iterator it = cylinder_map_.find(name);
  if (it == cylinder_map_.end()) 
  {
    printHiqpWarning("Couldn't update GeometricCylinder with name '" + 
      name + "'. The cylinder was not found!");
    return;
  }
  it->second->init(parameters);
}





template<>
void GeometricPrimitiveMap::updateGeometricPrimitive<GeometricSphere>
(
  const std::string& name, 
  const std::vector<double>& parameters
)
{
  SphereMap::iterator it = sphere_map_.find(name);
  if (it == sphere_map_.end()) 
  {
    printHiqpWarning("Couldn't update GeometricSphere with name '" + 
      name + "'. The sphere was not found!");
    return;
  }
  it->second->init(parameters);
}





template<>
void GeometricPrimitiveMap::updateGeometricPrimitive<GeometricFrame>
(
  const std::string& name, 
  const std::vector<double>& parameters
)
{
  FrameMap::iterator it = frame_map_.find(name);
  if (it == frame_map_.end()) 
  {
    printHiqpWarning("Couldn't update GeometricSphere with name '" + 
      name + "'. The sphere was not found!");
    return;
  }
  it->second->init(parameters);
}






////////////////////////////////////////////////////////////////////////////////
//  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  
////////////////////////////////////////////////////////////////////////////////
//
//                              P R I V A T E
//
////////////////////////////////////////////////////////////////////////////////

std::string GeometricPrimitiveMap::getDependenciesAsString
(
  std::string name
)
{
  DependencyMap::iterator it = dependency_map_.find(name);
  if (it != dependency_map_.end())
  {
    if (!it->second.empty())
    {
      std::stringstream ss;
      std::copy(it->second.begin(), it->second.end(),
        std::ostream_iterator< std::string >(ss, ", ")
      );
      return ss.str();
    }
  }
  return std::string();
}



} // namespace geometric_primitives

} // namespace hiqp


