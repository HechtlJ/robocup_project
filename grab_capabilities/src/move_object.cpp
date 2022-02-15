#include<move_object/move_object.h> 


namespace move_object
{  
MoveObject::MoveObject(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
{
  //-----------------------------initialization of menber variables----------------------------------------
  //object_detected_ = 0;
  grasp_processed_ = 0;
  flag_table_height_ = 0;
  empty_grasp_ = 0;     


  //-------------------------------publisher------------------------------------------
  head_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command",1);    
  gripper_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 1); 
 
  //-------------------------------subscriber----------------------------------------------
  nh_.setCallbackQueue(&queue_1); // set the callback queue 1
  // To receive the detected best grasp poses from GPD node
  object_posi_3d_ = nh_.subscribe<gpd_ros::GraspConfigList>("/detect_grasps/clustered_grasps",
                                                               1, &MoveObject::processGraspPose, this); 
  nh_.setCallbackQueue(&queue_2);  // set the callback queue 2 (not used in this project)
  sub_table_height_=nh_.subscribe<std_msgs::Float64>("/segmentation/table_height",
                                                     1, &MoveObject::getTableHeight, this);
}

MoveObject::~MoveObject(){}

void MoveObject::localization()
{
  ros::service::waitForService("/global_localization"); 
  ros::ServiceClient localization_client = nh_.serviceClient<std_srvs::Empty>("/global_localization");
  ros::service::waitForService("/move_base/clear_costmaps");
  ros::ServiceClient clean_costmap_client = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  ros::Publisher vel_pub = nh_.advertise<geometry_msgs::Twist>("mobile_base_controller/cmd_vel",100);

  std_srvs::Empty msg_empty;

  /***********************call the global_localization service**********************/
  if(localization_client.call(msg_empty))
  {
    ROS_INFO("Global localization has been successfully called");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to call the service 'global_localization'");
  }

  /*********Let Tiago turn left until covariance is close to zero for a robust localization*******/
  ros::Rate r(20);
  ros::Time begin=ros::Time::now();
  geometry_msgs::Twist cmd_vel;
  double max_cov=10;

  while(begin.toSec()==0)
  {
    begin = ros::Time::now();// to get the current time properly
  }
  
  ROS_INFO("Begin localization at time: %f",begin.toSec());
  covariance_space::CatchCovariance max_cov_class;

  nh_.setCallbackQueue(&queue_3);// set the callback queue 3
  ros::Subscriber cov_sub=nh_.subscribe("/amcl_pose",1,
                                       &covariance_space::CatchCovariance::covCallback,
                                       &max_cov_class);
  // turning until all absolut value of covariance is below 0.01
  while (max_cov > 0.01) 
  {
    queue_3.callOne(ros::WallDuration());
    ROS_INFO("localizing...");
    cmd_vel.angular.z = 0.9;
    vel_pub.publish(cmd_vel);
    max_cov = max_cov_class.getLocalMaxCov();
    ROS_INFO("max_cov:%f",max_cov);
    
    r.sleep();
  }

  ROS_INFO("localization done!");

  /***********************call the clear_costmap service**********************/
  if(clean_costmap_client.call(msg_empty))
    ROS_INFO("clean_costmap has been successfully called");
  else
    ROS_ERROR_STREAM("Failed to call the service 'clean_costmap'");
}

bool MoveObject::navigationTable1()
{  
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  MoveBaseClient ac("move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal table_1;
  table_1.target_pose.header.frame_id = "map";
  table_1.target_pose.header.stamp = ros::Time::now();
  table_1.target_pose.pose.position.x = 2.5;
  table_1.target_pose.pose.position.y = -0.465;
  table_1.target_pose.pose.orientation.x = 0;
  table_1.target_pose.pose.orientation.y = 0;
  table_1.target_pose.pose.orientation.z = 1;
  table_1.target_pose.pose.orientation.w = -0.07;
  ROS_INFO("Sending position of Table1");

  ac.sendGoal(table_1);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  { 
    ROS_INFO("Tiago reached Table 1, now is ready to pick up object");  
    return true;
  }
  else
  {
    ROS_INFO("Tiago failed to move to Table 1.");
    return false;
  }
}

void MoveObject::turnHeadSin()
{  
  //turning head command in sin mode
  trajectory_msgs::JointTrajectory msg_joint;
  msg_joint.joint_names.resize(2);
  msg_joint.joint_names[0] = "head_1_joint";
  msg_joint.joint_names[1] = "head_2_joint";
  msg_joint.points.resize(1);
  msg_joint.points[0].time_from_start =ros::Duration(0.2);
  msg_joint.points[0].positions.resize(2);
  double p1, p2;
  ros::Time time =ros::Time::now();
  p1 = 0.28 * sin(0.3 * time.toSec()) - 0.05;
  p2 = -0.3;
  msg_joint.points[0].positions[0] = p1; 
  msg_joint.points[0].positions[1] = p2; 
  head_control_.publish(msg_joint);
  }

void MoveObject::callServiceToFindGrasp()
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

void MoveObject::pickUpSetup()  
{
  // To save the time, Tiago will do the setup motion while waiting the arrival of the detected grasps
  ROS_INFO("Set Up 1:Open the gripper...");
  gripper_msg_.joint_names.resize(2);
  gripper_msg_.joint_names[0] = "gripper_left_finger_joint";
  gripper_msg_.joint_names[1] = "gripper_right_finger_joint";

  gripper_msg_.points.resize(1);
  gripper_msg_.points[0].positions.resize(2);
  gripper_msg_.points[0].effort.resize(2);

  gripper_msg_.points[0].positions[0] = 0.045;
  gripper_msg_.points[0].positions[1] = 0.045;

  gripper_msg_.points[0].effort[0] = 0;
  gripper_msg_.points[0].effort[1] = 0;

  gripper_msg_.points[0].time_from_start = ros::Duration(0.4);
    
  // publish the gripper control message
  gripper_pub_.publish(gripper_msg_);
  ROS_INFO("Set Up 2:Raise the arm......");

  //****************Add the table collision********
  // avoid to collide with the table
  pick_up_object_.addTableCollisionToWorld(0.4,0.5,"table1");
  // to cover the object on the table so that the arm will not collide with them
  pick_up_object_.addTableCollisionToWorld(0.56,0.25,"table_tall");
 
  double arm_pos1[8]={0.252, 0.6458, 0.5934, -1.7104, 1.7453, 1.8675,-0.0698,-1.0821};
  temp_joint_motion_.jointMotion(arm_pos1);  // Tiago will raise his arm and ready to pick up objects
}

bool MoveObject::pickUp()
{
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
  ROS_INFO_STREAM("motion plan disered grasp pose (orientation): x=" << ox << " y=" << oy << " z=" << oz<< " w=" << ow);
  
  // Add the approximated object collision to avoid colliding with the object while moving to the intermediate pose
  pick_up_object_.addObjectCollisionToWorld(collision_center_pick_,"temp_collison");

  //*********************pick up motion plan********************** 
  
  pick_up_object_.removeCollisionFromWorld("table_tall");
  ros::Duration(0.2).sleep();
  pick_up_object_.armTorsoMotion(grasp_pose_temp_);// move to the intermediate pose

  pick_up_object_.removeCollisionFromWorld("temp_collison");
  
  ros::Duration(0.2).sleep();

  pick_up_object_.armTorsoMotion(grasp_pose_);// move to the final grasp pose
 
  //------close the gripper------------------------
  gripper_msg_.joint_names.resize(2);
  gripper_msg_.joint_names[0] = "gripper_left_finger_joint";
  gripper_msg_.joint_names[1] = "gripper_right_finger_joint";

  gripper_msg_.points.resize(1);
  gripper_msg_.points[0].positions.resize(2);
  gripper_msg_.points[0].effort.resize(2);

  gripper_msg_.points[0].positions[0] = 0.0005;
  gripper_msg_.points[0].positions[1] = 0.0005;

  gripper_msg_.points[0].effort[0] = 80;
  gripper_msg_.points[0].effort[1] = 80;

  gripper_msg_.points[0].time_from_start = ros::Duration(1);
  // publish the gripper control message
  ROS_INFO("-------Close the gripper---------");
  gripper_pub_.publish(gripper_msg_);
  ros::Duration(1.2).sleep();

  //--------------return the arm to the initial state----------------
  double arm_pos_back_temp[8]={0.210, 2.6704, -0.0175, -1.6057, 1.2741, 0.2094,1.2741,-0.7156};
  double arm_pos_back[8] = {0.15,0.192,-1.34,-0.192,1.937,-1.37,1.25,0};

  geometry_msgs::PoseStamped grasp_pose_up_temp = grasp_pose_;
  grasp_pose_up_temp.pose.position.z += 0.20;

  pick_up_object_.armTorsoMotion(grasp_pose_up_temp);
  // Add a taller table collision so that the object in hand will not collide with the table while moving back to the tucked pose
  pick_up_object_.addTableCollisionToWorld(0.56,0.4,"table_tall2");
  temp_joint_motion_.jointMotion(arm_pos_back_temp);

  if (temp_joint_motion_.error_execute_plan_) // extreme case, selden happens 
  {
    pick_up_object_.removeCollisionFromWorld("table_tall2");
    temp_joint_motion_.jointMotion(arm_pos_back_temp);
  }
  temp_joint_motion_.jointMotion(arm_pos_back);
  pick_up_object_.removeCollisionFromWorld("table_tall2");
  
  ros::Duration(0.2).sleep();
  return true;
}

bool MoveObject::pickUpForGraspOnlyNode()
{
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
  ROS_INFO_STREAM("motion plan disered grasp pose (orientation): x=" << ox << " y=" << oy << " z=" << oz<< " w=" << ow);
  

  pick_up_object_.addObjectCollisionToWorld(collision_center_pick_,"temp_collison");

  //*********************pick up motion plan********************** 
  
  pick_up_object_.removeCollisionFromWorld("table_tall");
  ros::Duration(0.2).sleep();
  pick_up_object_.armTorsoMotion(grasp_pose_temp_);

  pick_up_object_.removeCollisionFromWorld("temp_collison");
  
  ros::Duration(0.2).sleep();

  //double arm_pos2[6]={x, y, z, 1.637, 0.466, -0.035};
  pick_up_object_.armTorsoMotion(grasp_pose_);
 
  //------close the gripper------------------------
  gripper_msg_.joint_names.resize(2);
  gripper_msg_.joint_names[0] = "gripper_left_finger_joint";
  gripper_msg_.joint_names[1] = "gripper_right_finger_joint";

  gripper_msg_.points.resize(1);
  gripper_msg_.points[0].positions.resize(2);
  gripper_msg_.points[0].effort.resize(2);

  gripper_msg_.points[0].positions[0] = 0.0005;
  gripper_msg_.points[0].positions[1] = 0.0005;

  gripper_msg_.points[0].effort[0] = 40;
  gripper_msg_.points[0].effort[1] = 40;

  gripper_msg_.points[0].time_from_start = ros::Duration(1.2);
  // publish the gripper control message
  ROS_INFO("-------Close the gripper---------");
  gripper_pub_.publish(gripper_msg_);
  ros::Duration(1.5).sleep();

  //--------------raise the arm----------------
  geometry_msgs::PoseStamped grasp_pose_up_temp = grasp_pose_;
  grasp_pose_up_temp.pose.position.z += 0.24;

  pick_up_object_.armTorsoMotion(grasp_pose_up_temp);

  return true;  
}

bool MoveObject::getInhand()
{
  
  tf::StampedTransform transform_fingers;
  tf_listener_.lookupTransform("gripper_left_finger_link","gripper_right_finger_link",ros::Time(0),transform_fingers);
  
  // get the distance between two frames based on the translation vector
  tf::Vector3 v_dist = transform_fingers.getOrigin(); 
  
  if (v_dist.length() > 0.005) // if the norm is bigger than the minimal distance of to finger frames, successfully hold in hand
  {
    ROS_INFO("object get in hand!");
    return true;
  }
  else
  {
    return false;
    ROS_INFO("didn't get object!");
  }

}

bool MoveObject::navigationTable2()
{
  bool flag_reached = false;
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  MoveBaseClient ac("move_base", true);
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal table_2;
  table_2.target_pose.header.frame_id = "map";
  table_2.target_pose.header.stamp = ros::Time::now();
  table_2.target_pose.pose.position.x = -0.61;
  table_2.target_pose.pose.position.y = -1.8;
  table_2.target_pose.pose.orientation.x = 0;
  table_2.target_pose.pose.orientation.y = 0;
  table_2.target_pose.pose.orientation.z = 1;
  table_2.target_pose.pose.orientation.w = -0.02;
  ROS_INFO("Sending position of Table 2");

  ac.sendGoal(table_2);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Tiago reached Table 2, now is ready to place the object");  
    return true;
  }
  else
  {
    ROS_INFO("Tiago failed to move to Table 2.");
    return false;
  }
}

void MoveObject::turnHeadDown()
{ 
  //turn head down
  trajectory_msgs::JointTrajectory msg_joint;
  msg_joint.joint_names.resize(2);
  msg_joint.joint_names[0] = "head_1_joint";
  msg_joint.joint_names[1] = "head_2_joint";
  msg_joint.points.resize(1);
  msg_joint.points[0].time_from_start =ros::Duration(0.6);
  msg_joint.points[0].positions.resize(2);

  msg_joint.points[0].positions[0] = -0.0; 
  msg_joint.points[0].positions[1] = -0.8;  
  head_control_.publish(msg_joint);
  ROS_INFO("Send head control command..");
}

bool MoveObject::putDown()
{
  //***************place the bottle properly according to the height of the table*************************
  //---draw the collision to avoid colliding with the walls and the carton sides
  put_down_object_.removeCollisionFromWorld("table1");
  put_down_object_.addTableCollisionToWorld(0.6, 0.06, "carton_side1", 1.3, 0.42);
  put_down_object_.addTableCollisionToWorld(1.5, 0.06, "carton_side2", 1.3, 0.86);
  put_down_object_.addTableCollisionToWorld(1.5, 1.3, "carton_side3", 0.06, 0, 0.8);
  
  //---move to the pose for landing the object
  double arm_pos_back_temp[8]={0.252, 0.6458, 0.5934, -1.7104, 1.7453, 1.8675,-0.0698,-1.0821};
  double arm_pos_place1[8] = {0.188,1.5882,0.2094,0.0524,1.6406,-1.4835,-0.384,0};
  //double arm_pos_place2[8] = {0.188,1.5882,0.2094,0.0524,1.6406,-1.4835,-0.384,1.7453};
  //double arm_pos_place3[8] = {0.188,1.5882,0.2094,0.0524,1.6406,-1.4835,-0.384,-1.396};

  temp_joint_motion_.jointMotion(arm_pos_back_temp);
  temp_joint_motion_.jointMotion(arm_pos_place1);

  //------open the gripper --------  
  gripper_msg_.points[0].positions[0] = 0.045;
  gripper_msg_.points[0].positions[1] = 0.045;

  // publish the gripper control message
  ROS_INFO("-------open the gripper---------");
  gripper_pub_.publish(gripper_msg_);
  ros::Duration(1).sleep();
  //-----sometimes the object can't drop down successfully, then turn around the gripper
  //temp_joint_motion_.jointMotion(arm_pos_place2);
  //temp_joint_motion_.jointMotion(arm_pos_place3);

  //------return the arm to the initial state------
  double arm_pos_back1[8] = {0.28,0.192,-1.34,-0.192,1.937,-1.57,1.361,0};
  double arm_pos_back2[8] = {0.15,0.192,-1.34,-0.192,1.937,-1.57,1.361,0};
  temp_joint_motion_.jointMotion(arm_pos_back1);
  temp_joint_motion_.jointMotion(arm_pos_back2);
  
  //-----remove the collision object
  put_down_object_.removeCollisionFromWorld("carton_side1");
  ros::Duration(0.1).sleep();
  put_down_object_.removeCollisionFromWorld("carton_side2");
  ros::Duration(0.1).sleep();
  put_down_object_.removeCollisionFromWorld("carton_side3");
  ros::Duration(0.1).sleep();
  return true;
}

bool MoveObject::putDownForGraspOnlyNode()
{
  pick_up_object_.addTableCollisionToWorld(0.60,0.55,"table_tall2",1.2); 
  ros::Duration(0.2).sleep();
  double arm_pos3[8]={0.156, 0.1571, 0.2792, 0.0349, 1.4661, -1.2915,0.1745,-0.1222};
  temp_joint_motion_.jointMotion(arm_pos3);

  pick_up_object_.removeCollisionFromWorld("table_tall2");
  //------open the gripper --------  
  gripper_msg_.points[0].positions[0] = 0.045;
  gripper_msg_.points[0].positions[1] = 0.045;

  // publish the gripper control message
  ROS_INFO("-------open the gripper---------");
  gripper_pub_.publish(gripper_msg_);
  ros::Duration(2).sleep();

  return true;
}
//************************private member function: callback functions ************************************//

void MoveObject::processGraspPose(const gpd_ros::GraspConfigListConstPtr& grasp_lst)
{
  //------------Processing the received grasp pose to get the final and intermediate grasp pose for Tiago
  double grasp_size = grasp_lst->grasps.size();
  if (grasp_size != 0)// for not empty plane case
  {
    //************************transform the 6-DOF to geometry_msg/PoseStamped type*************************
    geometry_msgs::Point mv_adjust;
    grasp_pose_.pose.position = grasp_lst->grasps[0].position;// get the position of the center of the gripper
    
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
    if(grasp_pose_.pose.position.x < 0.49|| mv_adjust.x < 0.08 || mv_adjust.z > 0.15) 
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

void MoveObject::getTableHeight (const std_msgs::Float64ConstPtr &msg)
{
  table_2_height_ = msg->data;
  flag_table_height_ = true;
} 

}

