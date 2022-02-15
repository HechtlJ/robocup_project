

#include <std_srvs/SetBool.h> 
#include <std_srvs/Empty.h> 
//#include <std_msgs/String.h>
#include <std_msgs/Float64.h> // for the height of the table
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <gpd_ros/GraspConfigList.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
// Std C++ headers
#include <string>
#include <vector>
#include <map>
#include <iostream>

//custum header file
#include <catch_covariance/catch_covariance.h>
#include <motion_plan_cartesian/motion_plan_cartesian.h>
#include <motion_plan_joint/motion_plan_joint.h>


namespace move_object
{
class MoveObject
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  
  // Define publishers and subscribers
  ros::Publisher head_control_;
  ros::Subscriber sub_detected_;
  ros::Publisher gripper_pub_;
  ros::Subscriber object_posi_3d_;
  ros::Subscriber sub_table_height_;
  // Define service client to call the service for sending the pre-processed PCL to GPD
  ros::ServiceClient send_pcl_client_;

  trajectory_msgs::JointTrajectory gripper_msg_; // msg for gripper close&open control

  // msg type to store the final grasp pose and the intermediate one
  geometry_msgs::PoseStamped grasp_pose_temp_;
  geometry_msgs::PoseStamped grasp_pose_;
 
  double table_2_height_;

  tf::TransformListener tf_listener_; // to look up transform between different frames
  
  //control in cartesian space: motion plan in cartesian space, add and remove collision to world
  motion_space::MotionPlanCartesian pick_up_object_; 
  motion_space::MotionPlanCartesian put_down_object_;  
  //control in joint space 
  motion_space::MotionPlanJoint temp_joint_motion_;  
  
  // msg type to store the center of the object collision
  geometry_msgs::Point collision_center_pick_;
  geometry_msgs::Point collision_center_place_;// not used in this project, may be useful for smart collision adding

  //------------------callback functions----------------//
  void processGraspPose(const gpd_ros::GraspConfigListConstPtr& msg);
  void getTableHeight(const std_msgs::Float64ConstPtr &msg);
 
public:
  //bool object_detected_ ;
  bool grasp_processed_;  // flag for judging whether grasp has been successfully proccessed
  bool empty_grasp_;     // flag for judging the empty table             
  bool flag_table_height_;// not used in this project, may be useful if the height of the table need to be detected

  // callback queues
  ros::CallbackQueue queue_1;
	ros::CallbackQueue queue_2;
  ros::CallbackQueue queue_3;

  

  MoveObject(ros::NodeHandle nh);
  ~MoveObject();
        
  void localization();
  bool navigationTable1(); 
  void turnHeadSin();
  void callServiceToFindGrasp();
  void pickUpSetup();           
  bool pickUp();
  bool pickUpForGraspOnlyNode(); // used for the simple demo version
  bool getInhand(); // judge whether Tiago successfully hold the object
  bool navigationTable2();
  void turnHeadDown(); 
  bool putDown();
  bool putDownForGraspOnlyNode(); // used for the simple demo version

};
}

#endif // ... MoveObject_H
