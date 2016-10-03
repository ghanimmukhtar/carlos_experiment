#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

//gazebo objects spawner
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <fstream>
#include <cmath>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/DeleteModel.h>

std::vector<Eigen::Vector3d> path, feedback_position, feedback_orientation, final_path, side_position(4), inter_point_position(4);
//the enumaration is ordinated to link cube side with corresponding outside point
enum CUBE_SIDE {RIGHT, LEFT, FRONT, BACK, NO_SIDE};
enum Inter_Points {Right_mid, Left_mid, Front_mid, Back_mid};

std::ofstream feedback_data;
std::vector<double> left_arm_joint_values(7), right_arm_joint_values(7);
std::vector <std::string> variable_names(19);
geometry_msgs::Pose table_pose, object_pose;
double cube_side = 0.06, inter_point_dist = 0.15, obj_pos_x, obj_pos_y;

/**
 * @brief jocommCallback: call back to get feedback about joints angles
 * @param the topic msgs about joints states
**/
void jocommCallback(sensor_msgs::JointState jo_state)
{
    left_arm_joint_values[0] = jo_state.position[5]; left_arm_joint_values[1] = jo_state.position[6]; left_arm_joint_values[2] = jo_state.position[3];
    left_arm_joint_values[3] = jo_state.position[4]; left_arm_joint_values[4] = jo_state.position[7]; left_arm_joint_values[5] = jo_state.position[8];
    left_arm_joint_values[6] = jo_state.position[9];

    right_arm_joint_values[0] = jo_state.position[14]; right_arm_joint_values[1] = jo_state.position[15]; right_arm_joint_values[2] = jo_state.position[12];
    right_arm_joint_values[3] = jo_state.position[13]; right_arm_joint_values[4] = jo_state.position[16]; right_arm_joint_values[5] = jo_state.position[17];
    right_arm_joint_values[6] = jo_state.position[18];
}

/**
 * @brief the function to spawn objects
 * @param name of the object to spawn, and its pose and the service client
 * @param argv
 * @return bool that sees if it succeed in spawning the model in gazebo or not
 */
bool spawn_my_model(std::string& model_to_spawn, ros::ServiceClient& my_spawner, geometry_msgs::Pose& model_pose){
    gazebo_msgs::SpawnModel my_model;
    std::string model_file_location = "/home/ghanim/.gazebo/models/" + model_to_spawn + "/model.sdf";
    std::ifstream model_file(model_file_location.c_str());
    std::string line;
    while (!model_file.eof()) {
        std::getline(model_file,line);
        my_model.request.model_xml+=line;
    }
    model_file.close();
    my_model.request.model_name = model_to_spawn;
    my_model.request.reference_frame = "base";
    my_model.request.initial_pose = model_pose;
    my_spawner.call(my_model);
    if(my_model.response.success)
        return true;
    else return false;
}
//find nearest vector from my_points to my_goal and return the index of this vector
int find_nearest(std::vector<Eigen::Vector3d>& my_points, Eigen::Vector3d& my_goal){
    double distance = std::numeric_limits<double>::infinity();
    Eigen::Vector3d distance_vector;
    int index_nearest;
    for (unsigned int i = 0; i < my_points.size(); ++i){
        distance_vector = my_goal - my_points[i];
        if (distance_vector.norm() < distance){
            distance = distance_vector.norm();
            index_nearest = i;
        }
    }
    return index_nearest;
}

//return roll pitch yaw from the transformation matrix transform_l_ee_w
Eigen::Vector3d extract_angles(Eigen::Matrix4d& transform_l_ee_w){
    Eigen::Vector3d my_angles;
    double Roll, Pitch, Yaw;
    Roll = atan2(transform_l_ee_w(1, 0), transform_l_ee_w(0, 0));
    Pitch = atan2(-transform_l_ee_w(2, 0), cos(Roll) * transform_l_ee_w(0, 0) + sin(Roll) * transform_l_ee_w(1, 0));
    Yaw = atan2(sin(Roll) * transform_l_ee_w(0, 2) - cos(Roll) * transform_l_ee_w(1, 2), -sin(Roll) * transform_l_ee_w(0, 1) + cos(Roll) * transform_l_ee_w(1, 1));
    my_angles << Roll, Pitch, Yaw;
    return my_angles;
}

//create joint trajectory with fixed orientation (Roll = 0, Pitch = M_PI, and Yaw = 0)
void create_trajectory_path(std::vector<Eigen::Vector3d>& the_path, ros::ServiceClient& ik_solver, trajectory_msgs::JointTrajectory& trajectory_path){
    geometry_msgs::PoseStamped my_desired_pose;
    my_desired_pose.header.frame_id = "/base";
    double ang_x = 0, ang_y = M_PI, ang_z = 0;
    tf::Quaternion my_orientation;
    trajectory_path.joint_names.clear();
    trajectory_path.points.clear();
    my_orientation.setRPY(ang_x, ang_y, ang_z);
    my_desired_pose.pose.orientation.w = my_orientation.getW();   my_desired_pose.pose.orientation.x = my_orientation.getX();    my_desired_pose.pose.orientation.y = my_orientation.getY();
    my_desired_pose.pose.orientation.z = my_orientation.getZ();
    if(ik_solver.exists()){
        for(int i = 0; i < the_path.size(); ++i){
            my_desired_pose.pose.position.x = the_path[i](0);
            my_desired_pose.pose.position.y = the_path[i](1);
            my_desired_pose.pose.position.z = the_path[i](2);
            //solve for desired position
            baxter_core_msgs::SolvePositionIK::Request req;
            baxter_core_msgs::SolvePositionIK::Response res;
            req.pose_stamp.push_back(my_desired_pose);

            ik_solver.call(req, res);

            ROS_WARN_STREAM("i am here11111111111");
            //if(res.isValid[0])
              //  ROS_WARN_STREAM("solution is found");
            //else
              //  ROS_ERROR("no solution found");
            if(!res.isValid.empty()){
                trajectory_msgs::JointTrajectoryPoint pt;
                pt.positions = res.joints[0].position;
                ROS_WARN_STREAM("i am here2222222222222");
                pt.velocities.resize(7, 0.0);
                pt.accelerations.resize(7, 0.0);
                pt.effort.resize(7, 0.0);
                pt.time_from_start = ros::Duration(i*1);
                if(!pt.positions.empty())
                   trajectory_path.points.push_back(pt);
            }
        }
        //at the end push joints names
        trajectory_path.joint_names = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};
    }
}

/**
 * @brief execute the descartes motion to a desired final position
 * @param desired position and ros handle
 * @param argv
 * @return
 */
bool goto_desired_position(Eigen::Vector3d goal, double rx, double ry, double rz, CUBE_SIDE my_side, ros::ServiceClient& ik_solver){
    Eigen::Vector3d my_inter_points;
    trajectory_msgs::JointTrajectory my_trajectory_path;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_left("/robot/limb/left/follow_joint_trajectory", true);
    //first get current position of end effector
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    robot_state::RobotState my_robot_state(robot_model);
    std::vector <std::string> variable_names(7);
    variable_names[0] = "left_s0"; variable_names[1] = "left_s1"; variable_names[2] = "left_e0";
    variable_names[3] = "left_e1"; variable_names[4] = "left_w0"; variable_names[5] = "left_w1";
    variable_names[6] = "left_w2";
    my_robot_state.setVariablePositions(variable_names, left_arm_joint_values);
    Eigen::Affine3d my_pose = my_robot_state.getGlobalLinkTransform("left_gripper");
    Eigen::Vector3d current_position;
    current_position = my_pose.translation();
    //create valid path
    if (my_side != NO_SIDE){
        path.clear(); final_path.clear();
        path.push_back(current_position);
        //find nearest intermediate point to starting position
        int index_nearest_goal, index_nearest, current_index;
        index_nearest_goal = find_nearest(inter_point_position, goal);
        path.push_back(inter_point_position[index_nearest_goal]);
        //add the final point which is the goal
        my_inter_points << goal(0), goal(1), goal(2);
        path.push_back(my_inter_points);
        //start constructing the fine trajectory
        final_path.push_back({path[0]});
        for (unsigned int k = 0; k < path.size() - 1; k++){
            for (unsigned int j = 0; j < 6; j++){
                Eigen::Vector3d apoint;
                apoint << path[k](0) + 0.2*j*(path[k + 1](0) - path[k](0)), path[k](1) + 0.2*j*(path[k + 1](1) - path[k](1)), path[k](2) + 0.2*j*(path[k + 1](2) - path[k](2));
                if (k == path.size() - 2 && j == 5)
                    apoint << path[k](0) + 0.3*j*(path[k + 1](0) - path[k](0)), path[k](1) + 0.3*j*(path[k + 1](1) - path[k](1)), path[k](2) + 0.2*j*(path[k + 1](2) - path[k](2));
                final_path.push_back(apoint);
            }
        }
        create_trajectory_path(final_path, ik_solver, my_trajectory_path);
    }
    //the arm is going to home position so just go in reverse
    else {
        path.clear();
        if ((current_position - goal).norm() < 0.01)
            return true;
        path.push_back(current_position);
        path.push_back(goal);
        create_trajectory_path(path, ik_solver, my_trajectory_path);
    }
    //execute the trajectory
    my_trajectory_path.header.stamp = ros::Time::now();
    if(my_trajectory_path.points.size() == 2)
        my_trajectory_path.points[1].time_from_start = ros::Duration(5.0);
    else if (my_trajectory_path.points.size() > 2){
        double start = 0.5;
        for(int i = 0; i < my_trajectory_path.joint_names.size(); ++i)
            my_trajectory_path.points[i].time_from_start = ros::Duration(start + i*start);
    }
    ROS_ERROR("I am trying to move!!!!!!!!!!!!!!");
    for(int i = 0; i < my_trajectory_path.joint_names.size(); ++i)
        std::cout << "joint name for joint: " << i << " is: " << my_trajectory_path.joint_names[i] << std::endl;
    if (!ac_left.waitForServer(ros::Duration(2.0)))
    {
      ROS_ERROR("Could not connect to action server");
      while (!ac_left.waitForServer(ros::Duration(2.0)))
          ROS_WARN_STREAM("Trying connect to action server");
    }
    control_msgs::FollowJointTrajectoryGoal traj_goal;
    traj_goal.trajectory = my_trajectory_path;

    traj_goal.goal_time_tolerance = ros::Duration(5);
    ac_left.sendGoal(traj_goal);
    while(!ac_left.getState().isDone());
    //if it reaches here it means everything went well
    return true;
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "carlos_experiment");
  ros::NodeHandle nh;
  ros::Subscriber sub_jointmsg = nh.subscribe<sensor_msgs::JointState>("/robot/joint_states",1,jocommCallback);
  ros::ServiceClient gazebo_spawn_clt = nh.serviceClient<gazebo_msgs::SpawnModel> ("/gazebo/spawn_sdf_model");
  ros::ServiceClient gazebo_model_state = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  ros::ServiceClient gazebo_model_delete = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
  ros::ServiceClient baxter_left_ik_solver = nh.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/left/PositionKinematicsNode/IKService");
  ros::ServiceClient baxter_right_ik_solver = nh.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/right/PositionKinematicsNode/IKService");
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_right("/robot/limb/right/follow_joint_trajectory", true);

  // Required for communication with moveit components
  ros::AsyncSpinner spinner (1);
  spinner.start();

  usleep(2e6);
  //move right arm to a safe position
  Eigen::Vector3d right_home = {0.0, -0.8, 0.2};
  double ang_x = 0, ang_y = M_PI, ang_z = 0;
  tf::Quaternion my_orientation;
  my_orientation.setRPY(ang_x, ang_y, ang_z);
  geometry_msgs::PoseStamped my_desired_pose;
  my_desired_pose.header.frame_id = "/base";
  my_desired_pose.pose.position.x = right_home(0);    my_desired_pose.pose.position.y = right_home(1);    my_desired_pose.pose.position.z = right_home(2);
  my_desired_pose.pose.orientation.w = my_orientation.getW();   my_desired_pose.pose.orientation.x = my_orientation.getX();    my_desired_pose.pose.orientation.y = my_orientation.getY();
  my_desired_pose.pose.orientation.z = my_orientation.getZ();
  //solve for desired position
  baxter_core_msgs::SolvePositionIK::Request req;
  baxter_core_msgs::SolvePositionIK::Response res;
  req.pose_stamp.push_back(my_desired_pose);
  baxter_right_ik_solver.call(req, res);
  if(!res.isValid[0])
      ROS_ERROR("no solution found");
  //create the sequence of joint trajectory points
  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names = res.joints[0].name;
  trajectory_msgs::JointTrajectoryPoint first_pt;
  first_pt.positions = right_arm_joint_values;
  first_pt.velocities.resize(7, 0.0);    first_pt.accelerations.resize(7, 0.0);    first_pt.effort.resize(7, 0.0);
  first_pt.time_from_start = ros::Duration(0.5);
  trajectory.points.push_back(first_pt);
  trajectory_msgs::JointTrajectoryPoint second_pt;
  second_pt.positions = res.joints[0].position;
  second_pt.velocities.resize(7, 0.0);    second_pt.accelerations.resize(7, 0.0);    second_pt.effort.resize(7, 0.0);
  second_pt.time_from_start = ros::Duration(5);
  trajectory.points.push_back(second_pt);
  for (int i = 0; i < trajectory.joint_names.size(); ++i)
      std::cout << "trajectory joints name for joint: " << i << " is: " << trajectory.joint_names[i] << std::endl;
  //move the arm using joint action server
  trajectory.header.stamp = ros::Time::now();
  if (!ac_right.waitForServer(ros::Duration(2.0)))
  {
    ROS_ERROR("Could not connect to action server");
  }
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration(5);
  ac_right.sendGoal(goal);
  while(!ac_right.getState().isDone());


  //spawn the table
  std::string objectName;
  my_orientation.setRPY(0, 0, 1.5707);
  table_pose.orientation.w = my_orientation.getW(); table_pose.orientation.z = my_orientation.getZ(); table_pose.orientation.y = my_orientation.getY(); table_pose.orientation.x = my_orientation.getX();
  table_pose.position.x = 0.7; table_pose.position.y = 0.0; table_pose.position.z = -0.9;
  //define starting point
  double home_x, home_y, home_z = 0.14, home_rx = 0.0, home_ry = M_PI, home_rz = 0.0, rx = 0.0, ry = M_PI, rz = 0.0;
  //pose of the cube
  object_pose.orientation.w = 1.0; object_pose.orientation.z = 0.0; object_pose.orientation.y = 0.0; object_pose.orientation.x = 0.0;
  object_pose.position.x = 0.65; object_pose.position.y = 0.1; object_pose.position.z = -0.1;
  side_position.clear(); inter_point_position.clear();
  //define coordinates of all points (object sides, and intermediate points)
  side_position.push_back({object_pose.position.x, object_pose.position.y - cube_side/2.0, object_pose.position.z}); //RIGHT
  side_position.push_back({object_pose.position.x, object_pose.position.y + cube_side/2.0, object_pose.position.z}); //LEFT
  side_position.push_back({object_pose.position.x + cube_side/2.0, object_pose.position.y, object_pose.position.z}); //FRONT
  side_position.push_back({object_pose.position.x - cube_side/2.0, object_pose.position.y, object_pose.position.z}); //BACK
  //Right_mid, Left_mid, Front_mid, Back_mid, Front_r, Front_l, Back_r, Back_l
  inter_point_position.push_back({object_pose.position.x, object_pose.position.y - cube_side/2.0 - inter_point_dist, -0.05}); //Right_mid
  inter_point_position.push_back({object_pose.position.x, object_pose.position.y + cube_side/2.0 + inter_point_dist, -0.05}); //Left_mid
  inter_point_position.push_back({object_pose.position.x - cube_side/2.0 - inter_point_dist, object_pose.position.y, -0.05}); //Front_mid
  inter_point_position.push_back({object_pose.position.x + cube_side/2.0 + inter_point_dist, object_pose.position.y, -0.05}); //Back_mid

  //open a file to register feedback data after/during each action
  feedback_data.open("/home/ghanim/git/data.csv");
  tf::Quaternion quat;
  //delete model
  gazebo_msgs::DeleteModel delete_model_cube, delete_model_table;
  delete_model_cube.request.model_name = "cube"; delete_model_table.request.model_name = "table";
  //each iteration change the initial position of the arm in a circle starting from 0.45, 0.1, 0.11
  int count = 0; double angle = 0, radius = 0.2, cx = object_pose.position.x, cy = object_pose.position.y;
  while(angle < 2*M_PI){
  //while(count < 2){
      //each iteration, respawn the object and move the arm to the selected side and push
      Eigen::Vector3d goal, home;
      home_x = cx + radius * cos(angle); home_y = cy + radius * sin(angle);
      home << home_x, home_y, home_z;
      int iteration = 0;
      while (iteration < 4){
          ROS_INFO("my X now is: %f and my Y is: %f", home_x, home_y);
          ROS_INFO("my angle now is: %f", angle);
          while(!goto_desired_position(home, home_rx, home_ry, home_rz, NO_SIDE, baxter_left_ik_solver));
          std::cin.ignore();
          CUBE_SIDE my_side = (CUBE_SIDE)iteration;
          goal = side_position[iteration];
          //CUBE_SIDE my_side = LEFT;
          std::cout << "side selected for this iteration is: " << my_side << std::endl;
          //spawn the table then the cube
          objectName = "table";
          spawn_my_model(objectName, gazebo_spawn_clt,table_pose);
          objectName = "cube";
          spawn_my_model(objectName,gazebo_spawn_clt,object_pose);

          //after spawning the object, get its dimension so the motion planning can be done
          gazebo_msgs::GetModelState model_state;
          model_state.request.model_name = "cube";
          gazebo_model_state.call(model_state);
          obj_pos_x = model_state.response.pose.position.x;
          obj_pos_y = model_state.response.pose.position.y;
          quat.setW(model_state.response.pose.orientation.w); quat.setX(model_state.response.pose.orientation.x);
          quat.setY(model_state.response.pose.orientation.y); quat.setZ(model_state.response.pose.orientation.z);
          double roll, pitch, yaw;
          tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
          feedback_data << model_state.response.pose.position.x << "," << model_state.response.pose.position.y << "," << model_state.response.pose.position.z
                        << "," << roll << "," << pitch << "," << yaw << ",";

          while(!goto_desired_position(goal, rx, ry, rz, my_side, baxter_left_ik_solver));
          std::cin.ignore();
          //get cube position after motion (action)
          gazebo_model_state.call(model_state);
          quat.setW(model_state.response.pose.orientation.w); quat.setX(model_state.response.pose.orientation.x);
          quat.setY(model_state.response.pose.orientation.y); quat.setZ(model_state.response.pose.orientation.z);
          tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
          feedback_data << model_state.response.pose.position.x << "," << model_state.response.pose.position.y << "," << model_state.response.pose.position.z
                        << "," << roll << "," << pitch << "," << yaw << ",";
          //save the path
          for (unsigned int h = 0; h < feedback_position.size(); ++h){
              feedback_data << feedback_position[h](0) << "," <<  feedback_position[h](1) << "," << feedback_position[h](2) << ","
                            << feedback_orientation[h](0) << "," << feedback_orientation[h](1) << "," << feedback_orientation[h](2) << ",";
          }
          feedback_data << "\n";
          gazebo_model_delete.call(delete_model_cube);
          gazebo_model_delete.call(delete_model_table);
          my_side = NO_SIDE;
          while(!goto_desired_position(home, home_rx, home_ry, home_rz, my_side, baxter_left_ik_solver));
          iteration+=1;
      }
      count+=1;
      //update starting point
      angle = angle + 0.63;
  }
  feedback_data.close();

  // Wait till user kills the process (Control-C)
  ROS_INFO("Done!");
  return 0;
}
