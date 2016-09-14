// Core ros functionality like ros::init and spin
#include <ros/ros.h>
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


typedef std::vector<descartes_core::TrajectoryPtPtr> TrajectoryVec;
typedef TrajectoryVec::const_iterator TrajectoryIter;
enum CUBE_SIDE {RIGHT, LEFT, FRONT, BACK, NO_SIDE};
enum Inter_Points {Front_mid, Front_r, Front_l, Right_mid, Left_mid, Back_mid, Back_r, Back_l};

std::vector<double> joints_values(19);
std::vector <std::string> variable_names(19);
geometry_msgs::Pose object_pose;
double cube_side = 0.1, inter_point_dist = 0.1, obj_pos_x, obj_pos_y;

void jocommCallback(sensor_msgs::JointState jo_state)
{
    for(int i = 0; i < jo_state.name.size(); i++){
        variable_names[i] = jo_state.name[i];
        joints_values[i] = jo_state.position[i];
    }
}

/**
 * Generates an completely defined (zero-tolerance) cartesian point from a pose
 */
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
 * Sends a ROS trajectory to the robot controller
 */
bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  // Create a Follow Joint Trajectory Action Client
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/robot/limb/left/follow_joint_trajectory", true);
  if (!ac.waitForServer(ros::Duration(2.0)))
  {
    ROS_ERROR("Could not connect to action server");
    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration(1.0);
  ac.sendGoal(goal);
  if (ac.waitForResult(goal.trajectory.points[goal.trajectory.points.size()-1].time_from_start + ros::Duration(5)))
  {
    ROS_INFO("Action server reported successful execution");
    return true;
  } else {
    ROS_WARN("Action server could not execute trajectory");
    return false;
  }
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
    executeTrajectory(result);
    executeTrajectory(result);
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
bool goto_desired_position(double x, double y, double z, double rx, double ry, double rz, CUBE_SIDE my_side, int iteration){

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


    if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
    {
      ROS_INFO("Could not initialize robot model");
      return false;
    }

    // 2. Define sequence of points
    //first get current position of end effector
    Eigen::Affine3d my_pose;
    Eigen::Vector3d current_position;
    const std::vector<double> left_arm_joint_values = {joints_values[5], joints_values[6], joints_values[3], joints_values[4], joints_values[7], joints_values[8], joints_values[9]};
    bool success = model->getFK(left_arm_joint_values,my_pose);
    if (success)
        current_position = my_pose.translation();
    else
        for(int i = 0; i < left_arm_joint_values.size(); i++)
            std::cout << "value of joint: " << i << " is: " << left_arm_joint_values[i] << std::endl;

    model->setCheckCollisions(true);
    //while (fabs(current_position(0) - home_x) > epsilon && fabs(current_position(1) - home_y) > epsilon && fabs(current_position(2) - home_z) > epsilon){

    //}

    double int_x, int_y, int_z = current_position(2);
    if (my_side == RIGHT || my_side == LEFT || my_side == FRONT || my_side == BACK){
        //Inter_Points my_intermediate_point = Inter_Points(rand()%8);
        Inter_Points my_intermediate_point;
        if (iteration == 0 || fmod(iteration, 8) == 0)
            my_intermediate_point = Front_mid;
        else if (iteration == 1 || fmod(iteration, 9) == 0)
            my_intermediate_point = Front_r;
        else if (iteration == 2 || fmod(iteration, 10) == 0)
            my_intermediate_point = Front_l;
        else if (iteration == 3 || fmod(iteration, 11) == 0)
            my_intermediate_point = Right_mid;
        else if (iteration == 4 || fmod(iteration, 12) == 0)
            my_intermediate_point = Left_mid;
        else if (iteration == 5 || fmod(iteration, 13) == 0)
            my_intermediate_point = Back_mid;
        else if (iteration == 6 || fmod(iteration, 14) == 0)
            my_intermediate_point = Back_r;
        else if (iteration == 7 || fmod(iteration, 15) == 0)
            my_intermediate_point = Back_l;

        switch (my_intermediate_point){
        case Front_mid: {int_x = obj_pos_x - cube_side/2.0 - inter_point_dist; int_y = obj_pos_y;}
            break;
        case Front_r: {int_x = obj_pos_x - cube_side/2.0 - inter_point_dist; int_y = obj_pos_y - cube_side/2.0 - inter_point_dist;}
            break;
        case Front_l: {int_x = obj_pos_x - cube_side/2.0 - inter_point_dist; int_y = obj_pos_y + cube_side/2.0 + inter_point_dist;}
            break;
        case Right_mid: {int_x = obj_pos_x; int_y = obj_pos_y - cube_side/2.0 - inter_point_dist;}
            break;
        case Left_mid: {int_x = obj_pos_x; int_y = obj_pos_y + cube_side/2.0 + inter_point_dist;}
            break;
        case Back_mid: {int_x = obj_pos_x + cube_side/2.0 + inter_point_dist; int_y = obj_pos_y;}
            break;
        case Back_r: {int_x = obj_pos_x + cube_side/2.0 + inter_point_dist; int_y = obj_pos_y - cube_side/2.0 - inter_point_dist;}
            break;
        case Back_l: {int_x = obj_pos_x + cube_side/2.0 + inter_point_dist; int_y = obj_pos_y + cube_side/2.0 + inter_point_dist;}
            break;
        default: ROS_WARN_STREAM("please enter valide intermediate point!!");
        }
        std::cout << "intermediate X is: " << int_x << std::endl
                    << "intermediate Y is: " << int_y << std::endl
                       << "intermediate Z is: " << int_z << std::endl;
    }

    TrajectoryVec points;
    if (my_side == RIGHT || my_side == LEFT || my_side == FRONT || my_side == BACK){
        for (unsigned int i = 0; i < 3; ++i)
        {
            descartes_core::TrajectoryPtPtr pt;
            if (i > 0){
                if(i == 1)
                    pt = makeTolerancedCartesianPoint(int_x, int_y, int_z, rx, ry, rz);
                else
                    pt = makeTolerancedCartesianPoint(x,y,z,rx,ry,rz);
            }
            else pt = makeTolerancedCartesianPoint2(my_pose);

          points.push_back(pt);
        }
    }
    else {
        for (unsigned int i = 0; i < 2; ++i)
        {
            descartes_core::TrajectoryPtPtr pt;
            if (i > 0){
                pt = makeTolerancedCartesianPoint(x,y,z,rx,ry,rz);
            }
            else pt = makeTolerancedCartesianPoint2(my_pose);

          points.push_back(pt);
        }
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

    //usleep(11e6);
    //executeTrajectory(joint_solution);
    //for some reason i need to do double try for execution with the real/gazebo simulated BAXTER robot
    /*executeTrajectory(joint_solution);
    if (!executeTrajectory(joint_solution))
    {
      ROS_ERROR("Could not execute trajectory!");
      return false;
    }
    else return true;*/
    return true;
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "carlos_experiment");
  ros::NodeHandle nh;
  ros::Subscriber sub_jointmsg;
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

  //delete model
  gazebo_msgs::DeleteModel delete_model_cube;
  delete_model_cube.request.model_name = "cube";
  //each iteration, respawn the object and move the arm to the selected side and push
  double home_x = 0.58, home_y = 0.18, home_z = 0.11, home_rx = 0.0, home_ry = M_PI, home_rz = 0.0, epsilon = 1e-3, x, y , z = -0.1, rx = 0.0, ry = M_PI, rz = 0.0;
  int iteration = 0;
  while (ros::ok()){
      //CUBE_SIDE my_side = CUBE_SIDE(rand()%4);
      CUBE_SIDE my_side = BACK;
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

      bool move_success = goto_desired_position(x, y, z, rx, ry, rz, my_side, iteration);
      my_side = NO_SIDE;
      move_success = goto_desired_position(home_x, home_y, home_z, home_rx, home_ry, home_rz, my_side, iteration);
      gazebo_model_delete.call(delete_model_cube);
      iteration+=1;
      ROS_WARN_STREAM("Hiiii I am im at iteration no:  ");
      std::cout << iteration << std::endl;
  }



  // Wait till user kills the process (Control-C)
  ROS_INFO("Done!");
  return 0;
}
