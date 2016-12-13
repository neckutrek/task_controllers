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

#include <hiqp/geometric_primitives/geometric_point.h>
#include <hiqp/geometric_primitives/geometric_cylinder.h>

#include <hiqp_ros/ros_topic_subscriber.h>

#include <hiqp_msgs/Vector3d.h>
#include <hiqp_msgs/StringArray.h>

#include <iostream>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <hiqp_msgs/PoseWithName.h>

namespace hiqp_ros
{

template<>
void ROSTopicSubscriber::topicCallback<hiqp_msgs::PoseWithName>(const hiqp_msgs::PoseWithName& msg) {

	double x = msg.pose.pose.position.x;
	if (x > 0.55) x = 0.55;
	if (x < 0) x = 0;

	double y = msg.pose.pose.position.y;
	if (y > 0.5) y = 0.5;
	if (y < -0.5) y = -0.5;

	double z = msg.pose.pose.position.z;
	if (z > 0.55) z = 0.55;
	if (z < 0.07) z = 0.07;

	std::vector<double> wintracker_frame_params;
	wintracker_frame_params.push_back(x);
	wintracker_frame_params.push_back(y);
	wintracker_frame_params.push_back(z);
	wintracker_frame_params.push_back(msg.pose.pose.orientation.w);
	wintracker_frame_params.push_back(msg.pose.pose.orientation.x);
	wintracker_frame_params.push_back(msg.pose.pose.orientation.y);
	wintracker_frame_params.push_back(msg.pose.pose.orientation.z);
	if(msg.name == "left") { //left hardcoded horrible hack
	    task_manager_->getGeometricPrimitiveMap()
	             ->updateGeometricPrimitive<GeometricFrame>("teleop_left_frame", wintracker_frame_params);
	} 
	if(msg.name == "right") { //right hardcoded horrible hack
	    task_manager_->getGeometricPrimitiveMap()
	             ->updateGeometricPrimitive<GeometricFrame>("teleop_right_frame", wintracker_frame_params);
	} 



	// BELOW IS OLD TELEOP CODE
	// std::vector<double> point_params;
	// point_params.push_back(msg.pose.position.x);
	// point_params.push_back(msg.pose.position.y);
	// point_params.push_back(msg.pose.position.z);
	// task_manager_->getGeometricPrimitiveMap()
	//              ->updateGeometricPrimitive<GeometricPoint>("teleop_point", point_params);
	// point_params.push_back(0.03);
	// task_manager_->getGeometricPrimitiveMap()
	//              ->updateGeometricPrimitive<GeometricSphere>("teleop_sphere", point_params);
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






template<>
void ROSTopicSubscriber::topicCallback<hiqp_msgs::Vector3d>
(
	const hiqp_msgs::Vector3d& msg
)
{
	/*
	GeometricCylinder* cylinder = primitive_map_->getGeometricPrimitive<GeometricCylinder>("thecylinder");

	if (cylinder == nullptr)
	{
		printHiqpWarning("Vector3d callback : couldn't find primitive 'thecylinder'");
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
	primitive_map_->updateGeometricPrimitive<GeometricCylinder>("thecylinder", params);

	std::cout << "Updated thecylinder\n";
	*/

}

template<>
void ROSTopicSubscriber::topicCallback<hiqp_msgs::StringArray>
(
	const hiqp_msgs::StringArray& msg
)
{
	if (msg.params.size() == 0) return;




	if (msg.params.at(0).compare("set_cyl_pos") == 0) {
		if (msg.params.size() != 4) return;
		double x = std::stod( msg.params.at(1) );
		double y = std::stod( msg.params.at(2) );
		double z = std::stod( msg.params.at(3) );

		std::shared_ptr<GeometricCylinder> cyl = 
			task_manager_->getGeometricPrimitiveMap()->getGeometricPrimitive
				<GeometricCylinder>("experiment_cylinder");
		std::vector<double> cyl_params = {
			cyl->getDirectionX(), cyl->getDirectionY(), cyl->getDirectionZ(), 
			x, y, z, cyl->getRadius(), cyl->getHeight()
		};
		cyl->init(cyl_params);




	} else if (msg.params.at(0).compare("goto_start_pos") == 0) {
		if (msg.params.size() != 4) return;

		double x = std::stod( msg.params.at(1) );
		double y = std::stod( msg.params.at(2) );
		double z = std::stod( msg.params.at(3) );

		std::shared_ptr<GeometricPoint> point = task_manager_->getGeometricPrimitiveMap()->getGeometricPrimitive
			<GeometricPoint>("experiment_starting_point");
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





