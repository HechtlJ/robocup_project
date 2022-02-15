#include "grabber.h"

Grabber::Grabber(ros::NodeHandle n)
{
  nh_ = n;

  //****************Add the table collision********
  // avoid to collide with the table
  addTableCollisionToWorld(0.4, 0.5, "table1");
  // to cover the object on the table so that the arm will not collide with them
  addTableCollisionToWorld(0.56, 0.25, "table_tall");

  // Services
  server_raise_arm_ = nh_.advertiseService("/grabber/raise_arm", &Grabber::raiseArm, this);
  server_pick_up_ = nh_.advertiseService("/grabber/pick_up", &Grabber::pickUp, this);

  // Publisher
  pub_gripper_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 1);

  // Subscriber
  sub_grasps = nh_.subscribe<gpd_ros::GraspConfigList>("/detect_grasps/clustered_grasps",
                                                       1, &Grabber::processGraspPose, this);
  ros::Subscriber sub_table_height;
}

int Grabber::armTorsoMotion(geometry_msgs::PoseStamped &arm_pos, const std::string &reference_frame)
{
  geometry_msgs::PoseStamped goal_pose;
  std::string ref_frame = reference_frame;

  // Set the goal_pose value
  goal_pose.header.frame_id = ref_frame;
  goal_pose.pose = arm_pos.pose;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // select group of joints
  moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
  // choose the preferred planner and the referent frame
  group_arm_torso.setPlannerId("SBLkConfigDefault");
  group_arm_torso.setPoseReferenceFrame(ref_frame);
  group_arm_torso.setPoseTarget(goal_pose);

  //-----------------find a motion plan------------------------

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1.0);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  group_arm_torso.setPlanningTime(2.0);
  bool success = bool(group_arm_torso.plan(my_plan));

  if (!success) // if failed to find a plan than try to find a plan with tolerance
  {
    ROS_INFO("Try to find a plan with tolerance");
    group_arm_torso.setGoalTolerance(0.05); // set the radius of the tolerance area
    bool success_to = bool(group_arm_torso.plan(my_plan));
    if (!success_to)
    {
      ROS_ERROR("Plan Not Found");
      spinner.stop();
      return false;
    }
    else
    {
      ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");
      //------------------Execute the plan---------------------------
      ros::Time start = ros::Time::now();
      moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();

      if (!bool(e))
      {
        ROS_ERROR("Error executing plan");
        spinner.stop();
        return false;
      }
      else
      {
        ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
        spinner.stop();
        return true;
      }
    }
  }
  else
  {
    ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");
    //------------------Execute the plan---------------------------
    ros::Time start = ros::Time::now();
    moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();

    if (!bool(e))
    {
      ROS_ERROR("Error executing plan");
      spinner.stop();
      return false;
    }
    else
    {
      ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
      spinner.stop();
      return true;
    }
  }
}

void Grabber::addTableCollisionToWorld(const double &table_height, const double &table_width, const std::string &table_name,
                                       const double &table_length, const double &table_distx, const double &table_disty)
{
  double height = table_height;
  double width = table_width;
  double length = table_length;
  double distance_x = table_distx;
  double distance_y = table_disty;
  std::string name = table_name;

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";
  collision_object.id = name;

  // Define the size of the box　
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = width;
  primitive.dimensions[1] = length;
  primitive.dimensions[2] = height - 0.1;

  // Define a pose for the box (specified relative to frame_id)　
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = distance_x;
  box_pose.position.y = distance_y;
  box_pose.position.z = (height - 0.1) * 0.5;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // Add collision to the world
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  ROS_INFO_STREAM("Now add a collision: " << name);
  planning_scene_interface_.addCollisionObjects(collision_objects);
  ros::Duration(0.3).sleep();
}
void Grabber::addObjectCollisionToWorld(geometry_msgs::Point &collision_center, const std::string &object_name,
                                        const std::string &object_type, const std::string &reference_frame)
{
  // double height = table_height;
  geometry_msgs::Point collision_ctr = collision_center;
  std::string name = object_name;
  std::string type = object_type;
  std::string rfr_frame = reference_frame;

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = rfr_frame;
  collision_object.id = name;
  shape_msgs::SolidPrimitive primitive;
  // Define the shape of the shape of the object
  if (type == "cylinder")
  {

    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 0.2;
    primitive.dimensions[1] = 0.06;
  }
  else // in default case, to add sphere collision
  {

    primitive.type = primitive.SPHERE;
    primitive.dimensions.resize(1);
    primitive.dimensions[0] = 0.08;
  }

  // Define a pose for the box (specified relative to frame_id)　
  geometry_msgs::Pose collison_pose;
  collison_pose.orientation.w = 1.0;
  collison_pose.position.x = collision_ctr.x;
  collison_pose.position.y = collision_ctr.y;
  collison_pose.position.z = collision_ctr.z;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(collison_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  ROS_INFO_STREAM("Now add a collision: " << name);
  planning_scene_interface_.addCollisionObjects(collision_objects);
  ros::Duration(0.3).sleep();
}
void Grabber::removeCollisionFromWorld(const std::string &remove_id)
{
  std::string rmv_id = remove_id;
  std::vector<std::string> object_ids;
  object_ids.push_back(rmv_id);
  ROS_INFO_STREAM("Now remove a collision: " << rmv_id);
  planning_scene_interface_.removeCollisionObjects(object_ids);
  ros::Duration(0.3).sleep();
}

void Grabber::jointMotion(double (&joint_position)[8])
{
  std::map<std::string, double> target_position;
  target_position["torso_lift_joint"] = joint_position[0];
  target_position["arm_1_joint"] = joint_position[1];
  target_position["arm_2_joint"] = joint_position[2];
  target_position["arm_3_joint"] = joint_position[3];
  target_position["arm_4_joint"] = joint_position[4];
  target_position["arm_5_joint"] = joint_position[5];
  target_position["arm_6_joint"] = joint_position[6];
  target_position["arm_7_joint"] = joint_position[7];

  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::vector<std::string> torso_arm_joint_names;
  // select group of joints
  moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
  // choose your preferred planner
  group_arm_torso.setPlannerId("SBLkConfigDefault");
  torso_arm_joint_names = group_arm_torso.getJoints();

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1.0);

  for (unsigned int i = 0; i < torso_arm_joint_names.size(); ++i)
    if (target_position.count(torso_arm_joint_names[i]) > 0)
    {
      ROS_INFO_STREAM("\t" << torso_arm_joint_names[i] << " goal position: " << target_position[torso_arm_joint_names[i]]);
      group_arm_torso.setJointValueTarget(torso_arm_joint_names[i], target_position[torso_arm_joint_names[i]]);
    }
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  group_arm_torso.setPlanningTime(2.0);

  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

  // Execute the plan　
  ros::Time start = ros::Time::now();
  moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();

  if (!bool(e))
  {
    ROS_ERROR("Error executing plan");
    spinner.stop();
    // error_execute_plan_ = true;
  }
  else
  {
    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
    spinner.stop();
    // error_execute_plan_ = false;
  }
}

bool Grabber::raiseArm(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
  double arm_pos1[8] = {0.252, 0.6458, 0.5934, -1.7104, 1.7453, 1.8675, -0.0698, -1.0821};
  jointMotion(arm_pos1); // Tiago will raise his arm and ready to pick up objects
  return true;
}

void Grabber::processGraspPose(const gpd_ros::GraspConfigListConstPtr &grasp_lst)
{
  //------------Processing the received grasp pose to get the final and intermediate grasp pose for Tiago
  double grasp_size = grasp_lst->grasps.size();
  if (grasp_size != 0) // for not empty plane case
  {
    //************************transform the 6-DOF to geometry_msg/PoseStamped type*************************
    geometry_msgs::Point mv_adjust;
    grasp_pose_.pose.position = grasp_lst->grasps[0].position; // get the position of the center of the gripper

    // Transform the 3-DOF orientation from the GPD node to quaternion form
    tf::Matrix3x3 orientation(
        grasp_lst->grasps[0].approach.x, grasp_lst->grasps[0].axis.x, -(grasp_lst->grasps[0].binormal.x),
        grasp_lst->grasps[0].approach.y, grasp_lst->grasps[0].axis.y, -(grasp_lst->grasps[0].binormal.y),
        grasp_lst->grasps[0].approach.z, grasp_lst->grasps[0].axis.z, -(grasp_lst->grasps[0].binormal.z));

    mv_adjust.x = grasp_lst->grasps[0].approach.x;
    mv_adjust.y = grasp_lst->grasps[0].approach.y;
    mv_adjust.z = grasp_lst->grasps[0].approach.z;

    ROS_INFO_STREAM("move adjust vector: x=" << mv_adjust.x << " y=" << mv_adjust.y << " z=" << mv_adjust.z);
    tf::Quaternion orientation_quat;
    orientation.getRotation(orientation_quat);
    orientation_quat.normalize();

    //----assign the value to the orientation of grasp_pose_ variable
    grasp_pose_.pose.orientation.x = orientation_quat.x();
    grasp_pose_.pose.orientation.y = orientation_quat.y();
    grasp_pose_.pose.orientation.z = orientation_quat.z();
    grasp_pose_.pose.orientation.w = orientation_quat.w();

    //*****************set the center of the object collision*******************
    collision_center_pick_.x = grasp_pose_.pose.position.x + 0.06 * mv_adjust.x;
    collision_center_pick_.y = grasp_pose_.pose.position.y + 0.06 * mv_adjust.y;
    collision_center_pick_.z = grasp_pose_.pose.position.z + 0.06 * mv_adjust.z;

    //*********Find the approperiate position for the origin of the end effector based on the gripper center*******
    double ad_factor = 0.1482;
    grasp_pose_.pose.position.x = grasp_pose_.pose.position.x - ad_factor * mv_adjust.x;
    grasp_pose_.pose.position.y = grasp_pose_.pose.position.y - ad_factor * mv_adjust.y;
    grasp_pose_.pose.position.z = grasp_pose_.pose.position.z - ad_factor * mv_adjust.z;

    grasp_pose_temp_ = grasp_pose_;
    double temp_factor = 0.13;
    //*******************Find the appropriate intermediate grasp pose *************************
    if (grasp_pose_.pose.position.x < 0.49 || mv_adjust.x < 0.08 || mv_adjust.z > 0.15)
    {
      // extreme cases: too close to the body, very trickt direction of approach
      double ad_factor_temp = 0.21;
      grasp_pose_temp_.pose.position.z = grasp_pose_temp_.pose.position.z + ad_factor_temp;
    }
    else
    {
      // normal case
      grasp_pose_temp_.pose.position.x = grasp_pose_.pose.position.x - temp_factor * mv_adjust.x;
      grasp_pose_temp_.pose.position.y = grasp_pose_.pose.position.y - temp_factor * mv_adjust.y;
      grasp_pose_temp_.pose.position.z = grasp_pose_.pose.position.z - temp_factor * mv_adjust.z;
    }
    grasp_processed_ = true;
  }
  else
  {
    empty_grasp_ = true;
    ROS_WARN("No objects on the table");
  }
}

bool Grabber::pickUp(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
  while (!grasp_processed_)
  {
    ros::spinOnce();
    ros::Duration(0.3).sleep();

    if (empty_grasp_) // detect the empty table
    {
      ROS_INFO("Please put some objects on the table"); // remind the user to put something on the table
      callServiceToFindGrasp();
      empty_grasp_ = false;
      ROS_INFO("Waiting for the coming grasp pose......");
    } //----
  }
  /***********************Start to pick up the object*******************/
  //------- move the arm to the object-----------
  double px, py, pz, ox, oy, oz, ow;
  // Show the pose of the selected grasp
  px = grasp_pose_.pose.position.x;
  py = grasp_pose_.pose.position.y;
  pz = grasp_pose_.pose.position.z;
  ox = grasp_pose_.pose.orientation.x;
  oy = grasp_pose_.pose.orientation.y;
  oz = grasp_pose_.pose.orientation.z;
  ow = grasp_pose_.pose.orientation.w;
  ROS_INFO_STREAM("motion plan disered grasp pose (position): x=" << px << " y=" << py << " z=" << pz);
  ROS_INFO_STREAM("motion plan disered grasp pose (orientation): x=" << ox << " y=" << oy << " z=" << oz << " w=" << ow);

  addObjectCollisionToWorld(collision_center_pick_, "temp_collison");

  //*********************pick up motion plan**********************

  removeCollisionFromWorld("table_tall");
  ros::Duration(0.2).sleep();
  armTorsoMotion(grasp_pose_temp_);

  removeCollisionFromWorld("temp_collison");

  ros::Duration(0.2).sleep();

  // double arm_pos2[6]={x, y, z, 1.637, 0.466, -0.035};
  armTorsoMotion(grasp_pose_);

  //------close the gripper------------------------
  trajectory_msgs::JointTrajectory gripper_msg;
  gripper_msg.joint_names.resize(2);
  gripper_msg.joint_names[0] = "gripper_left_finger_joint";
  gripper_msg.joint_names[1] = "gripper_right_finger_joint";

  gripper_msg.points.resize(1);
  gripper_msg.points[0].positions.resize(2);
  gripper_msg.points[0].effort.resize(2);

  gripper_msg.points[0].positions[0] = 0.0005;
  gripper_msg.points[0].positions[1] = 0.0005;

  gripper_msg.points[0].effort[0] = 40;
  gripper_msg.points[0].effort[1] = 40;

  gripper_msg.points[0].time_from_start = ros::Duration(1.2);
  // publish the gripper control message
  ROS_INFO("-------Close the gripper---------");
  pub_gripper_.publish(gripper_msg);
  ros::Duration(1.5).sleep();

  //--------------raise the arm----------------
  geometry_msgs::PoseStamped grasp_pose_up_temp = grasp_pose_;
  grasp_pose_up_temp.pose.position.z += 0.24;

  armTorsoMotion(grasp_pose_up_temp);

  return true;
}

void Grabber::callServiceToFindGrasp()
{
  //--------------------------------service client------------------------------------
  ros::service::waitForService("/segmentation/enable_send_pcl");
  send_pcl_client_ = nh_.serviceClient <std_srvs::SetBool> ("/segmentation/enable_send_pcl");

  std_srvs::SetBool enable_send_srv;
  bool if_success = false;
  enable_send_srv.request.data = true;
  do
  {
    ROS_INFO("Service: to enable sending transformed PCL");
    if_success = send_pcl_client_.call(enable_send_srv);
    ros::Duration(1).sleep();
  } while (!if_success);
}
