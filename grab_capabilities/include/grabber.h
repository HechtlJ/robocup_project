#pragma once

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>


#include <gpd_ros/GraspConfigList.h>
#include <tf/tf.h>


class Grabber
{
public:
    Grabber(ros::NodeHandle nh);

private:
    /************* Motion Plan *************/
    // Motion plan with tolerance in Cartesian space
    int armTorsoMotion(geometry_msgs::PoseStamped &arm_pos, const std::string &reference_frame="base_link");

    // Add box-form collision to the world, especially used for approximate table collision in this project
    void addTableCollisionToWorld(const double &table_height, const double &table_width, const std::string &table_name, 
                const double &table_length = 1.3, const double &table_distx = 0.68, const double &table_disty = 0);

    // Add Sphere or Cylinder-form of collision, especially used for approximate object collision in this project
    void addObjectCollisionToWorld(geometry_msgs::Point &collision_center, const std::string &object_name, const std::string &object_type = "sphere", 
                const std::string &reference_frame = "base_link");

    // Remove the collision from the world according to the collision id
    void removeCollisionFromWorld(const std::string &remove_id);

    void jointMotion(double (&joint_position)[8]);

    bool raiseArm(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
    bool pickUp(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);


    void callServiceToFindGrasp();


    // Callbacks
    void processGraspPose(const gpd_ros::GraspConfigListConstPtr& msg);

public:
private:
    ros::NodeHandle nh_;

    // Publisher
    ros::Publisher pub_gripper_;

    // Subscriber
    ros::Subscriber sub_grasps;
    ros::Subscriber sub_table_height;

    // Service Server
    ros::ServiceServer server_raise_arm_;
    ros::ServiceServer server_pick_up_;

    // Service Clients
    ros::ServiceClient send_pcl_client_;



    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;


    bool grasp_processed_;  // flag for judging whether grasp has been successfully proccessed
    bool empty_grasp_;     // flag for judging the empty table      


    // msg type to store the final grasp pose and the intermediate one
    geometry_msgs::PoseStamped grasp_pose_temp_;
    geometry_msgs::PoseStamped grasp_pose_;

    // msg type to store the center of the object collision
    geometry_msgs::Point collision_center_pick_;
};