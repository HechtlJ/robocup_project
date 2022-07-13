#pragma once

#include <vector>
#include  <iostream>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>


#include <gpd_ros/GraspConfigList.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>


/*********************************************************************
* darknet
********************************************************************/
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


#include "detection.h"

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
    void jointMotion(double (&joint_position)[8], double max_velocity_scaling_factor);


    // Services
    bool raiseArm(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
    bool pickUp(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
    bool srvIsObjectInHand(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
    bool srvPutObjectInBin(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
    bool srvIdentifyObject(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
    bool srvObservePreGrasp(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
    bool srvObserveAfterGrasp(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);


    void callServiceToFindGrasp();


    // Functions for controlling armcontinue
    void openGripper(bool blocking = true);
    void closeGripper(bool blocking = true);

    // check functions
    bool isObjectInHand();


    // Callbacks
    void processGraspPose(const gpd_ros::GraspConfigListConstPtr& msg);
    void detectionCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg);

public:
private:
    ros::NodeHandle nh_;

    // Publisher
    ros::Publisher pub_gripper_;

    // Subscriber
    ros::Subscriber sub_grasps;
    ros::Subscriber sub_table_height;
    ros::Subscriber sub_object_detections_;
    

    // Service Server
    ros::ServiceServer server_raise_arm_;
    ros::ServiceServer server_pick_up_;
    ros::ServiceServer server_object_in_hand_;
    ros::ServiceServer server_put_object_in_bin_;
    ros::ServiceServer server_identify_object_;
    ros::ServiceServer server_obs_pre_grasp_;
    ros::ServiceServer server_obs_after_grasp_;

    // Service Clients
    ros::ServiceClient send_pcl_client_;


    tf::TransformListener tf_listener_;

    std::string camera_frame_;                //!< camera frame name
    ros::Subscriber object_detections_sub_;   //!< sub detections form detector



    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;


    bool grasp_processed_;  // flag for judging whether grasp has been successfully proccessed
    bool empty_grasp_;     // flag for judging the empty table      


    // msg type to store the final grasp pose and the intermediate one
    geometry_msgs::PoseStamped grasp_pose_temp_;
    geometry_msgs::PoseStamped grasp_pose_;

    // msg type to store the center of the object collision
    geometry_msgs::Point collision_center_pick_;

    SceneObservation obs_current;
    SceneObservation obs_pre_grasp;
    SceneObservation obs_after_grasp;
    SceneObservation obs_inspect_object;

};