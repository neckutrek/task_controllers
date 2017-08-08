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

#include <hiqp/geometric_primitives/geometric_cylinder.h>
#include <hiqp/geometric_primitives/geometric_point.h>

#include <hiqp_ros/ros_topic_subscriber.h>

#include <hiqp_msgs/StringArray.h>
#include <hiqp_msgs/Vector3d.h>

#include <hiqp/utilities.h>
#include <hiqp/robot_state.h>

#include <iostream>
#include <string>
#include <vector>

#include <tf/tfMessage.h>
#include <geometry_msgs/PoseStamped.h>

namespace hiqp_ros {

///put all frames from tf as geometry frames in the primitve map
template <>
void ROSTopicSubscriber::topicCallback<tf::tfMessage>(
    const tf::tfMessage& msg) {
    
    for(int i=0; i<msg.transforms.size(); i++) {
	std::string parent_frame = msg.transforms[i].header.frame_id;
	std::string child_frame = msg.transforms[i].child_frame_id;
	std::vector<hiqp::PrimitiveInfo> prims = task_manager_->getGeometricPrimitiveMap()->getAllPrimitiveInfo();
	bool prim_exists = false;
	bool update_parent = false;
	bool parent_in_prim_map = false;
	
	//check if this primitve exists already and if the parent primitive is in the map
	for(int j=0; j<prims.size(); j++) {
	    if(prims[j].type.compare("frame")==0) {
		if(prims[j].name.compare(child_frame) == 0) {
		    prim_exists = true;
		    if(prims[j].frame_id.compare(parent_frame) != 0) {
			update_parent = true;
		    }
		}
		/*
		if(prims[j].name.compare(parent_frame) == 0 ) {
		    parent_in_prim_map = true;
		}
		*/
	    }
	}
	
	//try to lookup the child frame in the kdl tree. if it exists, we don't want to add it.
	if(hiqp::kdl_getQNrFromLinkName(robot_state_ptr_->kdl_tree_, child_frame) >= 0) {
	    //ROS_WARN("child frame %s is already in the kdl tree, skipping!", child_frame.c_str());
	    continue;
	} 
	//try to lookup parent frame in the kdl tree. if it exists, we can add the primitive
	if(hiqp::kdl_getQNrFromLinkName(robot_state_ptr_->kdl_tree_, parent_frame) < 0) {
	    //ROS_WARN("parent frame %s is not in the kdl tree!", parent_frame.c_str());
	    continue;
	    /*
	    //check parent is not in the geometry map either
	    if(!parent_in_prim_map) {
		ROS_WARN("parent frame %s is not in the primitive map, skipping!", parent_frame.c_str());
		continue;
	    }
	    */
	} 
	

	geometry_msgs::Transform t = msg.transforms[i].transform;
	std::vector<double> parameters { t.translation.x, t.translation.y, t.translation.z,
					 t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z };
	std::vector<double> color {0.8, 0.1, 0.8, 0.8};
	//if yes, we simply update the parameters
	if(prim_exists) {
	    if(update_parent) {
		//ROS_INFO("changing parent frame %s", child_frame.c_str());
		task_manager_->getGeometricPrimitiveMap()
		    ->getGeometricPrimitive<GeometricFrame>(child_frame)->resetFrameId(parent_frame);
	    
	    }
	    task_manager_->getGeometricPrimitiveMap()
		->updateGeometricPrimitive<GeometricFrame>(child_frame, parameters);
	    //ROS_INFO("updated frame %s",child_frame.c_str());
	    continue;
	}
	
	//if no, we add a new primitve
	task_manager_->getGeometricPrimitiveMap()->setGeometricPrimitive(child_frame, "frame",
									parent_frame, true, 
									color, parameters);
	ROS_INFO("added new frame %s",child_frame.c_str());
	
    }

}

template <>
void ROSTopicSubscriber::topicCallback<geometry_msgs::PoseStamped>(
    const geometry_msgs::PoseStamped& msg) {
  double x = msg.pose.position.x;
  if (x > 0.55) x = 0.55;
  if (x < 0) x = 0;

  double y = msg.pose.position.y;
  if (y > 0.5) y = 0.5;
  if (y < -0.5) y = -0.5;

  double z = msg.pose.position.z;
  if (z > 0.55) z = 0.55;
  if (z < 0.07) z = 0.07;

  std::vector<double> wintracker_frame_params;
  wintracker_frame_params.push_back(x);
  wintracker_frame_params.push_back(y);
  wintracker_frame_params.push_back(z);
  wintracker_frame_params.push_back(msg.pose.orientation.w);
  wintracker_frame_params.push_back(msg.pose.orientation.x);
  wintracker_frame_params.push_back(msg.pose.orientation.y);
  wintracker_frame_params.push_back(msg.pose.orientation.z);
  task_manager_->getGeometricPrimitiveMap()
      ->updateGeometricPrimitive<GeometricFrame>("teleop_wintracker_frame",
                                                 wintracker_frame_params);

  // BELOW IS OLD TELEOP CODE
  // std::vector<double> point_params;
  // point_params.push_back(msg.pose.position.x);
  // point_params.push_back(msg.pose.position.y);
  // point_params.push_back(msg.pose.position.z);
  // task_manager_->getGeometricPrimitiveMap()
  //              ->updateGeometricPrimitive<GeometricPoint>("teleop_point",
  //              point_params);
  // point_params.push_back(0.03);
  // task_manager_->getGeometricPrimitiveMap()
  //              ->updateGeometricPrimitive<GeometricSphere>("teleop_sphere",
  //              point_params);
  // std::cout << "pos = ("
  // 	<< msg.pose.position.x << ", "
  // 	<< msg.pose.position.y << ", "
  // 	<< msg.pose.position.z << ")\n"
  // 	<< "rot = ("
  // 	<< msg.pose.orientation.x << ", "
  // 	<< msg.pose.orientation.y << ", "
  // 	<< msg.pose.orientation.z << ", "
  // 	<< msg.pose.orientation.w << ")\n\n";
}

template <>
void ROSTopicSubscriber::topicCallback<hiqp_msgs::Vector3d>(
    const hiqp_msgs::Vector3d& msg) {
  /*
  GeometricCylinder* cylinder =
  primitive_map_->getGeometricPrimitive<GeometricCylinder>("thecylinder");

  if (cylinder == nullptr)
  {
          printHiqpWarning("Vector3d callback : couldn't find primitive
  'thecylinder'");
  }

  std::vector<double> params;
  params.push_back(cylinder->getDirectionX());
  params.push_back(cylinder->getDirectionY());
  params.push_back(cylinder->getDirectionZ());
  params.push_back(cylinder->getOffsetX() + msg.val1);
  params.push_back(cylinder->getOffsetY() + msg.val2);
  params.push_back(cylinder->getOffsetZ());
  params.push_back(cylinder->getRadius() + msg.val3);
  params.push_back(cylinder->getHeight());
  primitive_map_->updateGeometricPrimitive<GeometricCylinder>("thecylinder",
  params);

  std::cout << "Updated thecylinder\n";
  */
}

template <>
void ROSTopicSubscriber::topicCallback<hiqp_msgs::StringArray>(
    const hiqp_msgs::StringArray& msg) {
  if (msg.params.size() == 0) return;

  if (msg.params.at(0).compare("set_cyl_pos") == 0) {
    if (msg.params.size() != 4) return;
    double x = std::stod(msg.params.at(1));
    double y = std::stod(msg.params.at(2));
    double z = std::stod(msg.params.at(3));

    std::shared_ptr<GeometricCylinder> cyl =
        task_manager_->getGeometricPrimitiveMap()
            ->getGeometricPrimitive<GeometricCylinder>("experiment_cylinder");
    std::vector<double> cyl_params = {cyl->getDirectionX(),
                                      cyl->getDirectionY(),
                                      cyl->getDirectionZ(),
                                      x,
                                      y,
                                      z,
                                      cyl->getRadius(),
                                      cyl->getHeight()};
    cyl->init(cyl_params);

  } else if (msg.params.at(0).compare("goto_start_pos") == 0) {
    if (msg.params.size() != 4) return;

    double x = std::stod(msg.params.at(1));
    double y = std::stod(msg.params.at(2));
    double z = std::stod(msg.params.at(3));

    std::shared_ptr<GeometricPoint> point =
        task_manager_->getGeometricPrimitiveMap()
            ->getGeometricPrimitive<GeometricPoint>(
                "experiment_starting_point");
    std::vector<double> point_params = {x, y, z};
    point->init(point_params);

    task_manager_->deactivateTask("bring_back_to_start");
    task_manager_->deactivateTask("bring_gripper_point_to_cylinder");
    task_manager_->deactivateTask("bring_gripper_point_above_floor");
    task_manager_->deactivateTask("bring_gripper_point_under_plane");
    task_manager_->deactivateTask("align_gripper_with_floor");
    task_manager_->deactivateTask("align_gripper_with_cylinder");

    task_manager_->activateTask("bring_back_to_start");

  } else if (msg.params.at(0).compare("grab_cylinder") == 0) {
    task_manager_->deactivateTask("bring_back_to_start");
    task_manager_->deactivateTask("bring_gripper_point_to_cylinder");
    task_manager_->deactivateTask("bring_gripper_point_above_floor");
    task_manager_->deactivateTask("bring_gripper_point_under_plane");
    task_manager_->deactivateTask("align_gripper_with_floor");
    task_manager_->deactivateTask("align_gripper_with_cylinder");

    task_manager_->activateTask("bring_gripper_point_to_cylinder");
    task_manager_->activateTask("bring_gripper_point_above_floor");
    task_manager_->activateTask("bring_gripper_point_under_plane");
    task_manager_->activateTask("align_gripper_with_floor");
    task_manager_->activateTask("align_gripper_with_cylinder");
  }
}
}
