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

// move group
#include <moveit/move_group_interface/move_group.h>
#include <baxter_core_msgs/JointCommand.h>

//gazebo objects spawner
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <fstream>
#include <cmath>

//gazebo object info
//To Complete
#include <gazebo_msgs/GetModelState.h>

//gazebo deleting models
#include <gazebo_msgs/DeleteModel.h>

std::vector<Eigen::Vector3d> path, final_path;
typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
typedef TrajectoryVec::const_iterator TrajectoryIter;
//the enumaration is ordinated to link cube side with corresponding outside point
enum CUBE_SIDE {RIGHT, LEFT, FRONT, BACK, NO_SIDE};
enum Inter_Points {Right_mid, Left_mid, Front_mid, Back_mid, Front_r, Front_l, Back_r, Back_l};

std::ofstream feedback_data;
std::vector<double> joints_values(7);
std::vector <std::string> variable_names(19);
geometry_msgs::Pose object_pose;
double cube_side = 0.1, inter_point_dist = 0.1, obj_pos_x, obj_pos_y;

/**
 * @brief jocommCallback: call back to get feedback about joints angles
 * @param the topic msgs about joints states
**/
void jocommCallback(sensor_msgs::JointState jo_state)
{
    /*for(int i = 0; i < jo_state.name.size(); i++){
        variable_names[i] = jo_state.name[i];
        joints_values[i] = jo_state.position[i];
    }*/
    joints_values[0] = jo_state.position[5]; joints_values[1] = jo_state.position[6]; joints_values[2] = jo_state.position[3];
    joints_values[3] = jo_state.position[4]; joints_values[4] = jo_state.position[7]; joints_values[5] = jo_state.position[8];
    joints_values[6] = jo_state.position[9];
}

/**
 * Generates an completely defined (zero-tolerance) cartesian point from a pose
**/
descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d& pose)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose)) );
}

/**
 * Generates a cartesian point with free rotation about the Z axis of the EFF frame, given affine3 point or x, y, z, and rotation about the axes
 */
descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(double x, double y, double z, double rx, double ry, double rz)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
  //return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI/2.0-0.0001, AxialSymmetricPt::Z_AXIS) );
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
toROSJointTrajectory(const TrajectoryVec& trajectory,
                     const descartes_core::RobotModel& model,
                     const std::vector<std::string>& joint_names,
                     double time_delay)
{
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
    //executeTrajectory(result);
    //executeTrajectory(result);
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

/**
 * @brief execute the descartes motion to a desired final position
 * @param desired position and ros handle
 * @param argv
 * @return
 */
bool goto_desired_position(double x, double y, double z, double rx, double ry, double rz, CUBE_SIDE my_side, ros::Publisher& pub_msg){
    Eigen::Vector3d my_inter_points;
    // 1. Create a robot model and initialize it
    descartes_core::RobotModelPtr model (new descartes_moveit::MoveitStateAdapter);

    // Name of description on parameter server. Typically just "robot_description".
    const std::string robot_description = "robot_description";

    // name of the kinematic group you defined when running MoveitSetupAssistant
    const std::string group_name = "left_arm";

    // Name of frame in which you are expressing poses. Typically "world_frame" or "base_link".
    const std::string world_frame = "/base";

    // tool center point frame (name of link associated with tool)
    const std::string tcp_frame = "left_gripper";

    //moveit::planning_interface::MoveGroup group("left_arm");
    //moveit::planning_interface::MoveGroup::Plan my_plan;

    if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
    {
      ROS_INFO("Could not initialize robot model");
      return false;
    }

    // 2. Define sequence of points
    //first get current position of end effector
    Eigen::Affine3d my_pose;
    Eigen::Vector3d current_position;
    const std::vector<double> left_arm_joint_values = joints_values;
    //const std::vector<double> left_arm_joint_values = {joints_values[5], joints_values[6], joints_values[3], joints_values[4], joints_values[7], joints_values[8], joints_values[9]};
    bool success = model->getFK(left_arm_joint_values,my_pose);
    if (success)
        current_position = my_pose.translation();
    else
        for(int i = 0; i < left_arm_joint_values.size(); i++)
            std::cout << "value of joint: " << i << " is: " << left_arm_joint_values[i] << std::endl;
    if (my_side == RIGHT || my_side == LEFT || my_side == FRONT || my_side == BACK){
        path.clear(); final_path.clear();
    }
    path.push_back(current_position);
    model->setCheckCollisions(true);
    double int_x, int_y, int_z = current_position(2);
    if (my_side == RIGHT){
        //first point is front mid
        int_x = obj_pos_x - cube_side/2.0 - inter_point_dist; int_y = obj_pos_y;
        my_inter_points << int_x, int_y, int_z;
        path.push_back(my_inter_points);
        //second point is front right
        int_x = obj_pos_x - cube_side/2.0 - inter_point_dist; int_y = obj_pos_y - cube_side/2.0 - inter_point_dist;
        my_inter_points << int_x, int_y, int_z;
        path.push_back(my_inter_points);
        //third is right middle
        int_x = obj_pos_x; int_y = obj_pos_y - cube_side/2.0 - inter_point_dist;
        my_inter_points << int_x, int_y, int_z;
        path.push_back(my_inter_points);
        //lower the end effector
        my_inter_points << int_x, int_y, z;
        path.push_back(my_inter_points);
    }
    if (my_side == LEFT){
        //first point is front mid
        int_x = obj_pos_x - cube_side/2.0 - inter_point_dist; int_y = obj_pos_y;
        my_inter_points << int_x, int_y, int_z;
        path.push_back(my_inter_points);
        //second point is front left
        int_x = obj_pos_x - cube_side/2.0 - inter_point_dist; int_y = obj_pos_y + cube_side/2.0 + inter_point_dist;
        my_inter_points << int_x, int_y, int_z;
        path.push_back(my_inter_points);
        //third is left middle
        int_x = obj_pos_x; int_y = obj_pos_y + cube_side/2.0 + inter_point_dist;
        my_inter_points << int_x, int_y, int_z;
        path.push_back(my_inter_points);
        //lower the end effector
        my_inter_points << int_x, int_y, z;
        path.push_back(my_inter_points);
    }
    if (my_side == FRONT){
        //first point is front mid
        int_x = obj_pos_x - cube_side/2.0 - inter_point_dist; int_y = obj_pos_y;
        my_inter_points << int_x, int_y, int_z;
        path.push_back(my_inter_points);
        //lower the end effector
        my_inter_points << int_x, int_y, z;
        path.push_back(my_inter_points);
    }
    if (my_side == BACK){
        //first point is front mid
        int_x = obj_pos_x - cube_side/2.0 - inter_point_dist; int_y = obj_pos_y;
        my_inter_points << int_x, int_y, int_z;
        path.push_back(my_inter_points);
        //second point is front left
        int_x = obj_pos_x - cube_side/2.0 - inter_point_dist; int_y = obj_pos_y + cube_side/2.0 + inter_point_dist;
        my_inter_points << int_x, int_y, int_z;
        path.push_back(my_inter_points);
        //third is left middle
        int_x = obj_pos_x; int_y = obj_pos_y + cube_side/2.0 + inter_point_dist;
        my_inter_points << int_x, int_y, int_z;
        path.push_back(my_inter_points);
        //Four is back left
        int_x = obj_pos_x + cube_side/2.0 + inter_point_dist; int_y = obj_pos_y + cube_side/2.0 + inter_point_dist;
        my_inter_points << int_x, int_y, int_z;
        path.push_back(my_inter_points);
        //five is back middle
        int_x = obj_pos_x + cube_side/2.0 + inter_point_dist; int_y = obj_pos_y;
        my_inter_points << int_x, int_y, int_z;
        path.push_back(my_inter_points);
        //lower the end effector
        my_inter_points << int_x, int_y, z;
        path.push_back(my_inter_points);
    }


    TrajectoryVec points;
    if (my_side == RIGHT || my_side == LEFT || my_side == FRONT || my_side == BACK){
        my_inter_points << x, y, z;
        path.push_back(my_inter_points);
        final_path.push_back(path[0]);
        for (unsigned int k = 0; k < path.size() - 1; k++){
            for (unsigned int j = 0; j < 6; j++){
                Eigen::Vector3d apoint;
                apoint << path[k](0) + 0.2*j*(path[k + 1](0) - path[k](0)), path[k](1) + 0.2*j*(path[k + 1](1) - path[k](1)), path[k](2) + 0.2*j*(path[k + 1](2) - path[k](2));
                final_path.push_back(apoint);
            }
        }

        for (unsigned int i = 0; i < final_path.size(); ++i)
        {
            descartes_core::TrajectoryPtPtr pt;
            if (i > 0) pt = makeTolerancedCartesianPoint(final_path[i](0), final_path[i](1), final_path[i](2), rx, ry, rz);
            else pt = makeTolerancedCartesianPoint2(my_pose);
            points.push_back(pt);
        }
    }
    else {
        ROS_ERROR("!!!!!!!!!!!!! size of final path is: ");
        std::cout << final_path.size() << std::endl;
        for (int m = final_path.size() - 1; m > -1 ; --m)
        {
            //ROS_ERROR("!!!!!!!!!!!!! creating reverse path !!!!!!!!!!!!!!!!!!!!! ");
            descartes_core::TrajectoryPtPtr pt;
            pt = makeTolerancedCartesianPoint(final_path[m](0), final_path[m](1), final_path[m](2), rx, ry, rz);
                    // last point is home position
                    if (m == 0) pt = makeTolerancedCartesianPoint(x, y, z, rx, ry, rz);
            //else pt = makeTolerancedCartesianPoint2(my_pose);
            points.push_back(pt);
        }
        ROS_ERROR("!!!!!!!!!!!!! size of points for reverse is: ");
        std::cout << points.size() << std::endl;
    }

    //model->check_collisions_
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
    names = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};
    // Generate a ROS joint trajectory with the result path, robot model, given joint names,
    // a certain time delta between each trajectory point
    trajectory_msgs::JointTrajectory joint_solution = toROSJointTrajectory(result, *model, names, 1.0);
    baxter_core_msgs::JointCommand command_msg;
    command_msg.mode = command_msg.POSITION_MODE;
    command_msg.names = names;
    for (unsigned int n = 0; n < joint_solution.points.size(); ++n){
        command_msg.command = joint_solution.points[n].positions;
        std::vector<double> difference(7);
        for (unsigned int it = 0; it < joints_values.size(); ++it)
            difference[it] = joints_values[it] - joint_solution.points[n].positions[it];
        //std::vector<int>::iterator index_max = std::max_element(difference.begin(),difference.end()), index_min = std::min_element(difference.begin(),difference.end());
        double max_e = *std::max_element(difference.begin(),difference.end());
        double min_e = *std::min_element(difference.begin(),difference.end());
        while (std::max(fabs(max_e), fabs(min_e)) > 0.1){
            ROS_INFO("****************** Hiiiiiiiii here ************************ ");

            for (unsigned int it = 0; it < joints_values.size(); ++it)
                difference[it] = joints_values[it] - joint_solution.points[n].positions[it];
            max_e = *std::max_element(difference.begin(),difference.end());
            min_e = *std::min_element(difference.begin(),difference.end());
            pub_msg.publish(command_msg);
        }
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
  ros::Publisher pub_msg;
  pub_msg = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command",1);
  sub_jointmsg = nh.subscribe<sensor_msgs::JointState>("/robot/joint_states",1,jocommCallback);
  ros::ServiceClient gazebo_spawn_clt = nh.serviceClient<gazebo_msgs::SpawnModel> ("/gazebo/spawn_sdf_model");
  ros::ServiceClient gazebo_model_state = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  ros::ServiceClient gazebo_model_delete = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

  // Required for communication with moveit components
  ros::AsyncSpinner spinner (1);
  spinner.start();

  //spawn the table
  std::string objectName = "table";

  tf::Quaternion my_orientation;
  my_orientation.setRPY(0, 0, 1.5707);
  //object_pose.orientation.w = 1.0; object_pose.orientation.z = 1.2;
  object_pose.orientation.w = my_orientation.getW(); object_pose.orientation.z = my_orientation.getZ(); object_pose.orientation.y = my_orientation.getY(); object_pose.orientation.x = my_orientation.getX();
  object_pose.position.x = 0.7; object_pose.position.y = 0.0; object_pose.position.z = -0.9;
  bool spawn_success = spawn_my_model(objectName,gazebo_spawn_clt,object_pose);

  //open a file to register cube positions after each action
  tf::Quaternion quat;

  //delete model
  gazebo_msgs::DeleteModel delete_model_cube;
  delete_model_cube.request.model_name = "cube";
  //each iteration, respawn the object and move the arm to the selected side and push
  double home_x = 0.45, home_y = 0.1, home_z = 0.11, home_rx = 0.0, home_ry = M_PI, home_rz = 0.0, epsilon = 1e-3, x, y , z = -0.1, rx = 0.0, ry = M_PI, rz = 0.0;
  int iteration = 0;
  feedback_data.open("/home/ghanim/git/data.csv");
  while (iteration < 4){
      //srand (time(NULL));
      //CUBE_SIDE my_side = CUBE_SIDE(rand()%4);
      CUBE_SIDE my_side = (CUBE_SIDE)iteration;
      //CUBE_SIDE my_side = LEFT;
      std::cout << "side selected for this iteration is: " << my_side << std::endl;
      //spawn the cube
      objectName = "cube";
      object_pose.orientation.w = 1.0; object_pose.orientation.z = 0.0; object_pose.orientation.y = 0.0; object_pose.orientation.x = 0.0;
      object_pose.position.x = 0.65; object_pose.position.y = 0.1; object_pose.position.z = -0.1;
      spawn_success = spawn_my_model(objectName,gazebo_spawn_clt,object_pose);

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

      switch (my_side){
      case RIGHT : {x = obj_pos_x; y = obj_pos_y - cube_side/2.0;}
          break;
      case LEFT : {x = obj_pos_x; y = obj_pos_y + cube_side/2.0;}
          break;
      case FRONT: {x = obj_pos_x + cube_side/2.0; y = obj_pos_y;}
          break;
      case BACK: {x = obj_pos_x  - cube_side/2.0; y = obj_pos_y;}
          break;
      default: ROS_WARN_STREAM("give us a side please!!");
      }

      while(!goto_desired_position(x, y, z, rx, ry, rz, my_side, pub_msg));
      usleep(1e6);
      //get cube position after motion (action)
      gazebo_model_state.call(model_state);
      std::cout << "Current object X position is: " << model_state.response.pose.position.x << std::endl;
      std::cout << "Current object Y position is: " << model_state.response.pose.position.y << std::endl;
      std::cout << "Current object Z position is: " << model_state.response.pose.position.z << std::endl;
      quat.setW(model_state.response.pose.orientation.w); quat.setX(model_state.response.pose.orientation.x);
      quat.setY(model_state.response.pose.orientation.y); quat.setZ(model_state.response.pose.orientation.z);
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      feedback_data << model_state.response.pose.position.x << "," << model_state.response.pose.position.y << "," << model_state.response.pose.position.z
                    << "," << roll << "," << pitch << "," << yaw<< "\n";
      gazebo_model_delete.call(delete_model_cube);
      my_side = NO_SIDE;
      while(!goto_desired_position(home_x, home_y, home_z, home_rx, home_ry, home_rz, my_side, pub_msg));

      iteration+=1;
      ROS_WARN_STREAM("Hiiii I am im at iteration no:  ");
      std::cout << iteration << std::endl;

  }
  feedback_data.close();

  // Wait till user kills the process (Control-C)
  ROS_INFO("Done!");
  return 0;
}
