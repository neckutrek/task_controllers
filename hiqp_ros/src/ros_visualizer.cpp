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
#include <cmath>

#include <hiqp_ros/ros_visualizer.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

namespace hiqp_ros
{

  double marker_lifetime = 1; // added markers live for 1 second

  ROSVisualizer::ROSVisualizer() : next_id_(0) {}

  int ROSVisualizer::init(ros::NodeHandle* controller_nh) {
    controller_nh_ = controller_nh;
    marker_array_pub_ = controller_nh_->advertise<visualization_msgs::MarkerArray>
    ("visualization_marker", 1);
  }

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                A P P L Y                                   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

  int ROSVisualizer::apply(int id, std::shared_ptr<GeometricPoint> point, int action) {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/" + point->getFrameId();
    marker.header.stamp = ros::Time::now();
    marker.ns = kNamespace;
    if (action == ACTION_ADD)  marker.id = next_id_;
    else                       marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD; 

    marker.pose.position.x = point->getX();
    marker.pose.position.y = point->getY();
    marker.pose.position.z = point->getZ();

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 2*kPointRadius;
    marker.scale.y = 2*kPointRadius;
    marker.scale.z = 2*kPointRadius;

    marker.color.r = point->getRedComponent();
    marker.color.g = point->getGreenComponent();
    marker.color.b = point->getBlueComponent();
    marker.color.a = point->getAlphaComponent();

    marker.lifetime = ros::Duration(marker_lifetime);

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(marker);
    marker_array_pub_.publish(marker_array);

    if (action == ACTION_ADD) {
      next_id_++;
      return next_id_-1;
    } else {
      return id;
    }
  }

  int ROSVisualizer::apply(int id, std::shared_ptr<GeometricLine> line, int action) {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/" + line->getFrameId();
    marker.header.stamp = ros::Time::now();
    marker.ns = kNamespace;
    if (action == ACTION_ADD)  marker.id = next_id_;
    else                       marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD; 

    double length = kInfiniteLength;

    Eigen::Vector3d v = line->getDirectionEigen();
    Eigen::Vector3d p = line->getOffsetEigen();

    // Quaternion that aligns the z-axis with the line direction
    Eigen::Quaterniond q;
    q.setFromTwoVectors(Eigen::Vector3d::UnitZ(), v);

    marker.pose.position.x = p(0);
    marker.pose.position.y = p(1);
    marker.pose.position.z = p(2);

    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.scale.x = 2*kLineRadius;
    marker.scale.y = 2*kLineRadius;
    marker.scale.z = length;

    marker.color.r = line->getRedComponent();
    marker.color.g = line->getGreenComponent();
    marker.color.b = line->getBlueComponent();
    marker.color.a = line->getAlphaComponent();

    marker.lifetime = ros::Duration(marker_lifetime);

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(marker);
    marker_array_pub_.publish(marker_array);

    if (action == ACTION_ADD) {
      next_id_++;
      return next_id_-1;
    } else {
      return id;
    }
  }

  int ROSVisualizer::apply(int id, std::shared_ptr<GeometricPlane> plane, int action) {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/" + plane->getFrameId();
    marker.header.stamp = ros::Time::now();
    marker.ns = kNamespace;
    if (action == ACTION_ADD)  marker.id = next_id_;
    else                       marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD; 

    Eigen::Vector3d v = plane->getNormalEigen();
    Eigen::Vector3d p = plane->getOffset() * v;

    // Quaternion that aligns the z-axis with the line direction
    Eigen::Quaterniond q;
    q.setFromTwoVectors(Eigen::Vector3d::UnitZ(), v);

    marker.pose.position.x = p(0);
    marker.pose.position.y = p(1);
    marker.pose.position.z = p(2);

    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.scale.x = kInfiniteLength;
    marker.scale.y = kInfiniteLength;
    marker.scale.z = kPlaneThickness;

    marker.color.r = plane->getRedComponent();
    marker.color.g = plane->getGreenComponent();
    marker.color.b = plane->getBlueComponent();
    marker.color.a = plane->getAlphaComponent();

    marker.lifetime = ros::Duration(marker_lifetime);

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(marker);
    marker_array_pub_.publish(marker_array);

    if (action == ACTION_ADD) {
      next_id_++;
      return next_id_-1;
    } else {
      return id;
    }
  }


  #include <sstream>
  std::string dtostr(double d)
  {
    std::ostringstream strs;
    strs << d;
    std::string str = strs.str();
    return str;
  }

  int ROSVisualizer::apply(int id, std::shared_ptr<GeometricBox> box, int action) {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/" + box->getFrameId();
    marker.header.stamp = ros::Time::now();
    marker.ns = kNamespace;
    if (action == ACTION_ADD)  marker.id = next_id_;
    else                       marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD; 


    Eigen::Vector3d p = box->getCenterEigen();
    // Eigen::Vector3d nu = box->getNormalUpEigen();
    // Eigen::Vector3d nl = box->getNormalLeftEigen();

    // // Quaternion that aligns the x-axis with the normal of the upper side
    // // projected on the x-y plane
    // Eigen::Quaterniond q1;
    // Eigen::Vector3d nu_xy;
    // nu_xy << nu(0), nu(1), 0;
    // q1.setFromTwoVectors(Eigen::Vector3d::UnitX(), nu_xy);

    // // Quaternion that aligns the z-axis with the normal of the upper side
    // Eigen::Quaterniond q2;
    // q2.setFromTwoVectors(Eigen::Vector3d::UnitZ(), nu);

    // // Rotate around the normal upper side vector
    // Eigen::Vector3d left = Eigen::Vector3d::UnitZ().cross(nu);
    // Eigen::Quaterniond q3;
    // q3.setFromTwoVectors(left, nl);

    // Eigen::Quaterniond q = q1 * q2 * q3;


    Eigen::Quaterniond q = box->getQuaternionEigen();



    marker.pose.position.x = p(0);
    marker.pose.position.y = p(1);
    marker.pose.position.z = p(2);

    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.scale.x = box->getDimX();
    marker.scale.y = box->getDimY();
    marker.scale.z = box->getDimZ();

    marker.color.r = box->getRedComponent();
    marker.color.g = box->getGreenComponent();
    marker.color.b = box->getBlueComponent();
    marker.color.a = box->getAlphaComponent();

    marker.lifetime = ros::Duration(marker_lifetime);

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(marker);
    marker_array_pub_.publish(marker_array);

/*
    // For debugging purposes
    GeometricLine* line = new GeometricLine(
        "noname",  box->getFrameId(), true, {1,0,0,1},
        {dtostr(p1),dtostr(p2),dtostr(p3),dtostr(nl1),dtostr(nl2),dtostr(nl3)});
    apply(0, line, visualization_msgs::Marker::ADD);
*/

    if (action == ACTION_ADD) {
      next_id_++;
      return next_id_-1;
    } else {
      return id;
    }
  }

  int ROSVisualizer::apply(int id, std::shared_ptr<GeometricCylinder> cylinder, int action) {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/" + cylinder->getFrameId();
    marker.header.stamp = ros::Time::now();
    marker.ns = kNamespace;
    if (action == ACTION_ADD)  marker.id = next_id_;
    else                       marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD; 

    double height = (cylinder->isInfinite() ? 0 : cylinder->getHeight());

    Eigen::Vector3d v = cylinder->getDirectionEigen();
    Eigen::Vector3d p = cylinder->getOffsetEigen() + v*height/2;

    // Quaternion that aligns the z-axis with the line direction
    Eigen::Quaterniond q;
    q.setFromTwoVectors(Eigen::Vector3d::UnitZ(), v);

    marker.pose.position.x = p(0);
    marker.pose.position.y = p(1);
    marker.pose.position.z = p(2);

    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.scale.x = 2*cylinder->getRadius();
    marker.scale.y = 2*cylinder->getRadius();
    marker.scale.z = (cylinder->isInfinite() ? kInfiniteLength : height);

    marker.color.r = cylinder->getRedComponent();
    marker.color.g = cylinder->getGreenComponent();
    marker.color.b = cylinder->getBlueComponent();
    marker.color.a = cylinder->getAlphaComponent();

    marker.lifetime = ros::Duration(marker_lifetime);

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(marker);
    marker_array_pub_.publish(marker_array);

    if (action == ACTION_ADD) {
      next_id_++;
      return next_id_-1;
    } else {
      return id;
    }
  }

 int ROSVisualizer::apply(int id, std::shared_ptr<GeometricSphere> sphere, int action) {
   visualization_msgs::Marker marker;

   marker.header.frame_id = "/" + sphere->getFrameId();
   marker.header.stamp = ros::Time::now();
   marker.ns = kNamespace;
   if (action == ACTION_ADD)  marker.id = next_id_;
   else                       marker.id = id;
   marker.type = visualization_msgs::Marker::SPHERE;
   marker.action = visualization_msgs::Marker::ADD; 

   marker.pose.position.x = sphere->getX();
   marker.pose.position.y = sphere->getY();
   marker.pose.position.z = sphere->getZ();

   marker.pose.orientation.x = 0.0;
   marker.pose.orientation.y = 0.0;
   marker.pose.orientation.z = 0.0;
   marker.pose.orientation.w = 1.0;

   marker.scale.x = 2*sphere->getRadius();
   marker.scale.y = 2*sphere->getRadius();
   marker.scale.z = 2*sphere->getRadius();

   marker.color.r = sphere->getRedComponent();
   marker.color.g = sphere->getGreenComponent();
   marker.color.b = sphere->getBlueComponent();
   marker.color.a = sphere->getAlphaComponent();

   marker.lifetime = ros::Duration(marker_lifetime);

   visualization_msgs::MarkerArray marker_array;
   marker_array.markers.push_back(marker);
   marker_array_pub_.publish(marker_array);

   if (action == ACTION_ADD) {
     next_id_++;
     return next_id_-1;
   } else {
     return id;
   }
 }

 int ROSVisualizer::apply(int id, std::shared_ptr<GeometricFrame> frame, int action) {
  visualization_msgs::MarkerArray marker_array;
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/" + frame->getFrameId();
    marker.header.stamp = ros::Time::now();
    marker.ns = kNamespace;
    if (action == ACTION_ADD)  marker.id = next_id_;
    else                       marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD; 
    marker.pose.position.x = frame->getX();
    marker.pose.position.y = frame->getY();
    marker.pose.position.z = frame->getZ();
    marker.pose.orientation.x = frame->getQX();
    marker.pose.orientation.y = frame->getQY();
    marker.pose.orientation.z = frame->getQZ();
    marker.pose.orientation.w = frame->getQW();
    marker.scale.x = kFrameArrowLength;
    marker.scale.y = 2*kFrameArrowRadius;
    marker.scale.z = 2*kFrameArrowRadius;
    marker.color.r = 1.0;//0.5*frame->getRedComponent() + 0.5;
    marker.color.g = 0.0;//0.5*frame->getGreenComponent();
    marker.color.b = 0.0;//0.5*frame->getBlueComponent();
    marker.color.a = frame->getAlphaComponent();
    marker.lifetime = ros::Duration(marker_lifetime);
    marker_array.markers.push_back(marker);
  }
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/" + frame->getFrameId();
    marker.header.stamp = ros::Time::now();
    marker.ns = kNamespace;
    if (action == ACTION_ADD)  marker.id = next_id_+1;
    else                       marker.id = id+1;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD; 
    marker.pose.position.x = frame->getX();
    marker.pose.position.y = frame->getY();
    marker.pose.position.z = frame->getZ();
    Eigen::Quaternion<double> rotq(0.70710678118, 0, 0, 0.70710678118);
    Eigen::Quaternion<double> resq = frame->getQuaternionEigen()*rotq;
    marker.pose.orientation.x = resq.x();
    marker.pose.orientation.y = resq.y();
    marker.pose.orientation.z = resq.z();
    marker.pose.orientation.w = resq.w();
    marker.scale.x = kFrameArrowLength;
    marker.scale.y = 2*kFrameArrowRadius;
    marker.scale.z = 2*kFrameArrowRadius;
    marker.color.r = 0.0;//0.5*frame->getRedComponent() + 0.5;
    marker.color.g = 1.0;//0.5*frame->getGreenComponent();
    marker.color.b = 0.0;//0.5*frame->getBlueComponent();
    marker.color.a = frame->getAlphaComponent();
    marker.lifetime = ros::Duration(marker_lifetime);
    marker_array.markers.push_back(marker);
  }
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/" + frame->getFrameId();
    marker.header.stamp = ros::Time::now();
    marker.ns = kNamespace;
    if (action == ACTION_ADD)  marker.id = next_id_+2;
    else                       marker.id = id+2;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD; 
    marker.pose.position.x = frame->getX();
    marker.pose.position.y = frame->getY();
    marker.pose.position.z = frame->getZ();
    Eigen::Quaternion<double> rotq(0.70710678118, 0, -0.70710678118, 0);
    Eigen::Quaternion<double> resq = frame->getQuaternionEigen()*rotq;
    marker.pose.orientation.x = resq.x();
    marker.pose.orientation.y = resq.y();
    marker.pose.orientation.z = resq.z();
    marker.pose.orientation.w = resq.w();
    marker.scale.x = kFrameArrowLength;
    marker.scale.y = 2*kFrameArrowRadius;
    marker.scale.z = 2*kFrameArrowRadius;
    marker.color.r = 0.0;//0.5*frame->getRedComponent() + 0.5;
    marker.color.g = 0.0;//0.5*frame->getGreenComponent();
    marker.color.b = 1.0;//0.5*frame->getBlueComponent();
    marker.color.a = frame->getAlphaComponent();
    marker.lifetime = ros::Duration(marker_lifetime);
    marker_array.markers.push_back(marker);
  }

  marker_array_pub_.publish(marker_array);

  if (action == ACTION_ADD) {
    next_id_ += 3;
    return next_id_-3;
  } else {
    return id;
  }
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                 A D D                                      //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

int ROSVisualizer::add(std::shared_ptr<GeometricPoint> point) {
  return apply(0, point, ACTION_ADD);
}

int ROSVisualizer::add(std::shared_ptr<GeometricLine> line) {
  return apply(0, line, ACTION_ADD);
}

int ROSVisualizer::add(std::shared_ptr<GeometricPlane> plane) {
  return apply(0, plane, ACTION_ADD);
}

int ROSVisualizer::add(std::shared_ptr<GeometricBox> box) {
  return apply(0, box, ACTION_ADD);
}

int ROSVisualizer::add(std::shared_ptr<GeometricCylinder> cylinder) {
  return apply(0, cylinder, ACTION_ADD);
}

int ROSVisualizer::add(std::shared_ptr<GeometricSphere> sphere) {
  return apply(0, sphere, ACTION_ADD);
}

int ROSVisualizer::add(std::shared_ptr<GeometricFrame> frame) {
  return apply(0, frame, ACTION_ADD);
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                              U P D A T E                                   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

void ROSVisualizer::update(int id, std::shared_ptr<GeometricPoint> point) {
  apply(id, point, ACTION_MODIFY);
}

void ROSVisualizer::update(int id, std::shared_ptr<GeometricLine> line) {
  apply(id, line, ACTION_MODIFY);
}

void ROSVisualizer::update(int id, std::shared_ptr<GeometricPlane> plane) {
  apply(id, plane, ACTION_MODIFY);
}

void ROSVisualizer::update(int id, std::shared_ptr<GeometricBox> box) {
  apply(id, box, ACTION_MODIFY);
}

void ROSVisualizer::update(int id, std::shared_ptr<GeometricCylinder> cylinder) {
  apply(id, cylinder, ACTION_MODIFY);
}

void ROSVisualizer::update(int id, std::shared_ptr<GeometricSphere> sphere) {
  apply(id, sphere, ACTION_MODIFY);
}

void ROSVisualizer::update(int id, std::shared_ptr<GeometricFrame> frame) {
  apply(id, frame, ACTION_MODIFY);
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                              R E M O V E                                   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

void ROSVisualizer::remove(int id) {
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.ns = kNamespace;
  marker.id = id;
  marker.action = visualization_msgs::Marker::DELETE; 
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.push_back(marker);
  marker_array_pub_.publish(marker_array);
}


void ROSVisualizer::removeMany(const std::vector<int>& ids) {
  std::cout << "removeMany: ";

  visualization_msgs::MarkerArray marker_array;
  for (int id : ids) {
    std::cout << id << ", ";
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.ns = kNamespace;
    marker.id = id;
    marker.action = visualization_msgs::Marker::DELETE; 
    marker_array.markers.push_back(marker);
  }
  std::cout << "\n";
  marker_array_pub_.publish(marker_array);
}

} // namespace hiqp