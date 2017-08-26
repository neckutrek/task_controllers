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

#ifndef HIQP_ROS_VISUALIZER_H
#define HIQP_ROS_VISUALIZER_H

#include <hiqp/visualizer.h>

#include <ros/ros.h>

using hiqp::Visualizer;
using hiqp::geometric_primitives::GeometricPoint;
using hiqp::geometric_primitives::GeometricLine;
using hiqp::geometric_primitives::GeometricPlane;
using hiqp::geometric_primitives::GeometricBox;
using hiqp::geometric_primitives::GeometricCylinder;
using hiqp::geometric_primitives::GeometricSphere;
using hiqp::geometric_primitives::GeometricFrame;

namespace hiqp_ros {

/// \todo Make node handle pointer a shared pointer
/*! \brief
 *  \author Marcus A Johansson */
class ROSVisualizer : public hiqp::Visualizer {
 public:
  ROSVisualizer();
  ~ROSVisualizer() noexcept {}

  int init(ros::NodeHandle* controller_nh);

  int add(std::shared_ptr<GeometricPoint> point);
  int add(std::shared_ptr<GeometricLine> line);
  int add(std::shared_ptr<GeometricPlane> plane);
  int add(std::shared_ptr<GeometricBox> box);
  int add(std::shared_ptr<GeometricCylinder> cylinder);
  int add(std::shared_ptr<GeometricSphere> sphere);
  int add(std::shared_ptr<GeometricFrame> frame);

  void update(int id, std::shared_ptr<GeometricPoint> point);
  void update(int id, std::shared_ptr<GeometricLine> line);
  void update(int id, std::shared_ptr<GeometricPlane> plane);
  void update(int id, std::shared_ptr<GeometricBox> box);
  void update(int id, std::shared_ptr<GeometricCylinder> cylinder);
  void update(int id, std::shared_ptr<GeometricSphere> sphere);
  void update(int id, std::shared_ptr<GeometricFrame> frame);

  void remove(int id);

  void removeMany(const std::vector<int>& ids);

 private:
  ROSVisualizer(const ROSVisualizer& other) = delete;
  ROSVisualizer(ROSVisualizer&& other) = delete;
  ROSVisualizer& operator=(const ROSVisualizer& other) = delete;
  ROSVisualizer& operator=(ROSVisualizer&& other) noexcept = delete;

  int apply(int id, std::shared_ptr<GeometricPoint> point, int action);
  int apply(int id, std::shared_ptr<GeometricLine> line, int action);
  int apply(int id, std::shared_ptr<GeometricPlane> plane, int action);
  int apply(int id, std::shared_ptr<GeometricBox> box, int action);
  int apply(int id, std::shared_ptr<GeometricCylinder> cylinder, int action);
  int apply(int id, std::shared_ptr<GeometricSphere> sphere, int action);
  int apply(int id, std::shared_ptr<GeometricFrame> frame, int action);

  enum { ACTION_ADD = 0, ACTION_MODIFY = 1 };

  const std::string kNamespace = "/yumi";
  const double kInfiniteLength = 12;
  const double kPointRadius = 0.005;
  const double kLineRadius = 0.0005;
  const double kPlaneThickness = 0.001;
  const double kFrameArrowRadius = 0.0017;
  const double kFrameArrowLength = 0.04;

  ros::NodeHandle* controller_nh_;

  ros::Publisher marker_array_pub_;

  std::size_t next_id_;
};

}  // namespace hiqp

#endif  // include guard