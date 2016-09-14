// Core ros functionality like ros::init and spin
#include <ros/ros.h>
#include <iostream>
#include <fstream>
// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>

// Includes the descartes robot model we will be using
#include <descartes_moveit/moveit_state_adapter.h>
// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
// Includes the planner we will be using
#include <descartes_planner/dense_planner.h>

#include <baxter_core_msgs/JointCommand.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <fstream>
#include <cmath>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/DeleteModel.h>

std::vector<Eigen::Vector3d> path, feedback_position, feedback_orientation, final_path, side_position(4), inter_point_position(4);
typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
typedef TrajectoryVec::const_iterator TrajectoryIter;
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
 * Generates a cartesian point with free rotation about the Z axis of the EFF frame, given affine3 point or x, y, z, and rotation about the axes
 */
descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(double x, double y, double z, double rx, double ry, double rz)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
    return TrajectoryPtPtr( new AxialSymmetricPt(x,y,z,rx,ry,rz, M_PI/2.0-0.0001, AxialSymmetricPt::Z_AXIS) );
}

descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint2(const Eigen::Affine3d& pose)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
  return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI/2.0-0.0001, AxialSymmetricPt::Z_AXIS) );
}

/**
 * Translates a descartes trajectory to a ROS joint trajectory
 */
trajectory_msgs::JointTrajectory
toROSJointTrajectory(const TrajectoryVec& trajectory, const descartes_core::RobotModel& model, const std::vector<std::string>& joint_names, double time_delay){
  // Fill out information about our trajectory
  trajectory_msgs::JointTrajectory result;
  result.header.stamp = ros::Time::now();
  result.header.frame_id = "world_frame";
  result.joint_names = joint_names;

  // For keeping track of time-so-far in the trajectory
  double time_offset = 0.0;
  // Loop through the trajectory
  for (TrajectoryIter it = trajectory.begin(); it != trajectory.end(); ++it)
  {
    // Find nominal joint solution at this point
    std::vector<double> joints;
    it->get()->getNominalJointPose(std::vector<double>(), model, joints);
    // Fill out a ROS trajectory point
    trajectory_msgs::JointTrajectoryPoint pt;
    for(int i = 0; i < joints.size(); i++)
        pt.positions.push_back(joints[i]);

    // velocity, acceleration, and effort are given dummy values
    // we'll let the controller figure them out
    pt.velocities.resize(7, 0.0);
    pt.accelerations.resize(7, 0.0);
    pt.effort.resize(7, 0.0);
    // set the time into the trajectory
    pt.time_from_start = ros::Duration(time_offset);
    // increment time
    time_offset += time_delay;
    result.points.push_back(pt);
  }
  return result;
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

/**
 * @brief execute the descartes motion to a desired final position
 * @param desired position and ros handle
 * @param argv
 * @return
 */
bool goto_desired_position(std::string limb, Eigen::Vector3d goal, double rx, double ry, double rz, CUBE_SIDE my_side, ros::Publisher& pub_msg){
    Eigen::Vector3d my_inter_points;
    // 1. Create a robot model and initialize it
    descartes_core::RobotModelPtr model (new descartes_moveit::MoveitStateAdapter);

    // Name of description on parameter server. Typically just "robot_description".
    const std::string robot_description = "robot_description";
    // Name of frame in which you are expressing poses. Typically "world_frame" or "base_link".
    const std::string world_frame = "/base";

    // name of the kinematic group you defined when running MoveitSetupAssistant
    if (limb == "left"){
        const std::string group_name = "left_arm";
        const std::string tcp_frame = "left_gripper";
        if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
        {
          ROS_INFO("Could not initialize robot model");
          return false;
        }
    }
    else{
        const std::string group_name = "right_arm";
        const std::string tcp_frame = "right_gripper";
        if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
        {
          ROS_INFO("Could not initialize robot model");
          return false;
        }
    }

    model->setCheckCollisions(true);
    // 2. Define sequence of points
    //first get current position of end effector
    Eigen::Affine3d my_pose;
    Eigen::Vector3d current_position;
    if (limb == "left")
        while(!model->getFK(left_arm_joint_values,my_pose));
    else
        while(!model->getFK(right_arm_joint_values,my_pose));
    current_position = my_pose.translation();
    //create valid path
    TrajectoryVec points;
    if (my_side != NO_SIDE){
        path.clear(); final_path.clear();
        path.push_back(current_position);
        //find nearest intermediate point to starting position
        int index_nearest_goal, index_nearest, current_index;
        index_nearest_goal = find_nearest(inter_point_position, goal);
        path.push_back(inter_point_position[index_nearest_goal]);
        /*index_nearest = find_nearest(inter_point_position, current_position);
        //current_index = index_nearest;
        //for (int i = 0; i < inter_point_position.size(); ++i)
            //std::cout << inter_point_position[i] << std::endl;
        Eigen::Vector3d last_path_point = inter_point_position[index_nearest];
        path.push_back(last_path_point);
        //find nearest intermediate point to the goal
        index_nearest_goal = find_nearest(inter_point_position, goal);
        //now starting from the nearest intermediate point to current position try to get to the goal
        std::vector<Eigen::Vector3d> updated_map = inter_point_position;
        updated_map.erase(updated_map.begin() + current_index);
        while (last_path_point(0) != inter_point_position[index_nearest_goal](0) ||
               last_path_point(1) != inter_point_position[index_nearest_goal](1) ||
               last_path_point(2) != inter_point_position[index_nearest_goal](2)){
            //ROS_INFO("Hi i am here !!!!!!!!!!!!!!!");
            //std::cout << "new point is: \n" << last_path_point << std::endl << "*******************************" << std::endl;
            int candid_current_index = find_nearest(updated_map, inter_point_position[current_index]);
            if (candid_current_index == find_nearest(updated_map, inter_point_position[index_nearest_goal])){
                current_index = candid_current_index;
                last_path_point = updated_map[current_index];
                std::cout << "new point is: \n" << last_path_point << std::endl << "*******************************" << std::endl;
                path.push_back(last_path_point);
                updated_map.erase(updated_map.begin() + current_index);
            }
            else updated_map.erase(updated_map.begin() + candid_current_index);;
        }
        //add the point in level of cube side
        path.push_back({path[path.size() - 1](0), path[path.size() - 1](1), goal(2)});
        */
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
        //final_path = path;

        //create the tolerance points
        for (unsigned int i = 0; i < final_path.size(); ++i)
        {
            descartes_core::TrajectoryPtPtr pt;
            if (i > 0) pt = makeTolerancedCartesianPoint(final_path[i](0), final_path[i](1), final_path[i](2), rx, ry, rz);
            else pt = makeTolerancedCartesianPoint2(my_pose);
            points.push_back(pt);
        }
    }
    //the arm is going to home position so just go in reverse
    else {
        path.clear();
        if ((current_position - goal).norm() < 0.01)
            return true;
        for (unsigned int i = 0; i < 2; ++i)
        {
            descartes_core::TrajectoryPtPtr pt;
            if (i > 0){
                pt = makeTolerancedCartesianPoint(goal(0),goal(1),goal(2),rx,ry,rz);
            }
            else pt = makeTolerancedCartesianPoint2(my_pose);
          points.push_back(pt);
        }
    }
    // 3. Create a planner and initialize it with our robot model
    descartes_planner::DensePlanner planner;
    planner.initialize(model);
    // 4. Feed the trajectory to the planner
    if (!planner.planPath(points))
    {
      ROS_ERROR("Could not solve for a valid path");
      return false;
    }

    TrajectoryVec result;
    if (!planner.getPath(result))
    {
      ROS_ERROR("Could not retrieve path");
      return false;
    }

    // 5. Translate the result into a type that ROS understands
    // Get Joint Names
    std::vector<std::string> names;
    if(limb == "left")
        names = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};
    else
        names = {"right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"};
    // Generate a ROS joint trajectory with the result path, robot model, given joint names,
    // a certain time delta between each trajectory point

    trajectory_msgs::JointTrajectory joint_solution = toROSJointTrajectory(result, *model, names, 1.0);
    baxter_core_msgs::JointCommand command_msg;
    command_msg.mode = command_msg.POSITION_MODE;
    command_msg.names = names;
    feedback_position.clear(); feedback_orientation.clear();
    for (unsigned int n = 0; n < joint_solution.points.size(); ++n){
        command_msg.command = joint_solution.points[n].positions;
        std::vector<double> difference(7), joints_values(7);
        if (limb == "left") joints_values = left_arm_joint_values;
        else joints_values = right_arm_joint_values;
        for (unsigned int it = 0; it < joints_values.size(); ++it)
            difference[it] = joints_values[it] - joint_solution.points[n].positions[it];
        double max_e = *std::max_element(difference.begin(),difference.end());
        double min_e = *std::min_element(difference.begin(),difference.end());
        while (std::max(fabs(max_e), fabs(min_e)) > 0.01){
            //ROS_INFO("the difference is: %f", std::max(fabs(max_e), fabs(min_e)));
            if (limb == "left") joints_values = left_arm_joint_values;
            else joints_values = right_arm_joint_values;
            //each 6 step it should be the same point as in the path, so get the eef pose and safe it
            for (unsigned int it = 0; it < joints_values.size(); ++it)
                difference[it] = joints_values[it] - joint_solution.points[n].positions[it];
            max_e = *std::max_element(difference.begin(),difference.end());
            min_e = *std::min_element(difference.begin(),difference.end());
            usleep(1e3);
            pub_msg.publish(command_msg);
        }
        if (my_side != NO_SIDE && limb == "left"){
            if (n%6 == 0){
                while(!model->getFK(left_arm_joint_values, my_pose));
                feedback_position.push_back(my_pose.translation());
                Eigen::Matrix4d transform_l_ee_w = my_pose.matrix();
                Eigen::Vector3d my_angles = extract_angles(transform_l_ee_w);
                feedback_orientation.push_back(my_angles);
            }
        }
        usleep(1e3);
    }
    //if it reaches here it means everything went well
    return true;
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "carlos_experiment");
  ros::NodeHandle nh;
  ros::Subscriber sub_jointmsg;
  ros::Publisher pub_msg_left, pub_msg_right;
  pub_msg_left = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command",1);
  pub_msg_right = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command",1);
  sub_jointmsg = nh.subscribe<sensor_msgs::JointState>("/robot/joint_states",1,jocommCallback);
  ros::ServiceClient gazebo_spawn_clt = nh.serviceClient<gazebo_msgs::SpawnModel> ("/gazebo/spawn_sdf_model");
  ros::ServiceClient gazebo_model_state = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  ros::ServiceClient gazebo_model_delete = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

  // Required for communication with moveit components
  ros::AsyncSpinner spinner (8);
  spinner.start();

  usleep(2e6);
  //move right arm to a safe position
  Eigen::Vector3d right_home = {0.0, -1.0, 0.1};
  while(!goto_desired_position("right", right_home, 0, M_PI, 0, NO_SIDE, pub_msg_right));

  //spawn the table
  std::string objectName;
  tf::Quaternion my_orientation;
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
          while(!goto_desired_position("left",home, home_rx, home_ry, home_rz, NO_SIDE, pub_msg_left));
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

          while(!goto_desired_position("left", goal, rx, ry, rz, my_side, pub_msg_left));
          usleep(1e6);
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
          while(!goto_desired_position("left",home, home_rx, home_ry, home_rz, my_side, pub_msg_left));
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
