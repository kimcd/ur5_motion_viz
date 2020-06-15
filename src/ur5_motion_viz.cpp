/* 
UR5_MOTION_VIZ node subscribes to /traj_data. The incoming trajectory
is checked for self-collision on the UR5. The output is a ROS_INFO message 
indicating whether a particular set of joint data is in self-collision or not.

This code borrows heavily from planning_scene_tutorial.cpp 

TODO: 
-turn into a service 
-report results of collision to a topic as a boolean 
-remove dependence on ur5_moveit_config demo.launch 

C. KIM, JHUAPL 14June2020
*/

#include <pluginlib/class_loader.h> //what do?
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <iterator>

// MoveIt!
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h> // what do?
#include <moveit/planning_interface/planning_interface.h> // what do?
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h> // added to read /traj_data
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit/collision_detection/collision_tools.h>
#include <boost/scoped_ptr.hpp>

planning_scene::PlanningScene* g_planning_scene = 0; // null pointer

void traj_dataCallback(const moveit_msgs::RobotTrajectory& msg)
{
  ROS_INFO("I heard ");
  std::cout << "Total set of data points: " << msg.joint_trajectory.points.size() << ".\n";
  trajectory_msgs::JointTrajectory traj_data_msg; // declare a variable as JointTrajectory type to access data 
  
  // Access Elements of traj_data_msg
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  traj_data_msg.joint_names = msg.joint_trajectory.joint_names;   
  std::cout << traj_data_msg;

  // Setup Planning Group
  // ^^^^^^^^^^^^^^^^^^^^
  // Create a RobotState and JointModelGroup to keep track of the 
  // current robot pose and planning group. 
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
  const std::string PLANNING_GROUP = "manipulator";
  const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
  g_planning_scene = new planning_scene::PlanningScene(robot_model);
  g_planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

  // Collision Check Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // Create collision request and result objects. Planning scene maintains the current state
  // internally. We can get a reference to it and change it and check for collisions.
  collision_detection::CollisionRequest c_req;
  collision_detection::CollisionResult c_res; 
  robot_state::RobotState& current_state = g_planning_scene->getCurrentStateNonConst();
  
  // Access and Parse traj_data
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^
  std::vector<int>::size_type size1 = msg.joint_trajectory.points.size(); 
    for (unsigned i = 0; i<size1; i++)
    {
	//std::cout <<msg.joint_trajectory.points[i].positions[0] << ".\n";
        std::vector<double> joint_values = {msg.joint_trajectory.points[i].positions[0], msg.joint_trajectory.points[i].positions[1], msg.joint_trajectory.points[i].positions[2], msg.joint_trajectory.points[i].positions[3], msg.joint_trajectory.points[i].positions[4], msg.joint_trajectory.points[i].positions[5]};
        
	//Print Joint Data
	//^^^^^^^^^^^^^^^^ 
        std::cout << "Joint Trajectory Set " << i+1 << ": ["; 
        for(int i=0; i < joint_values.size(); i++)
          std::cout <<joint_values.at(i) << ' '; 
          std::cout << "] \n"; 
	
	//Collision Checking
	//^^^^^^^^^^^^^^^^^^
	const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup(PLANNING_GROUP); 
	current_state.setJointGroupPositions(joint_model_group, joint_values);
	c_req.contacts = true; 
	c_req.max_contacts = 1000;
	c_res.clear(); 
  	g_planning_scene->checkSelfCollision(c_req, c_res); 
  	ROS_INFO_STREAM("Current state is " << (c_res.collision ? "in" : "not in") << " self collision");
     } 	  
}

void computeCollisionContactPoints(robot_model::RobotModelPtr& robot_model)
{

}
 
int main(int argc, char** argv)
{
  const std::string node_name = "ur5_motion_viz";
  ros::init(argc, argv, node_name);
  ros::NodeHandle node_handle;  

  ros::AsyncSpinner spinner(0);
  spinner.start();

  ros::Subscriber sub = node_handle.subscribe("traj_data", 10, traj_dataCallback);
  std::cout << "Hello" << ".\n";
 
  ros::waitForShutdown();
}
