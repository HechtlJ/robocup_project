/**
 * @file image_channel_converter_node.cpp
 * @author Johannes Hechtl
 * @brief converts RGB images from IMAGE_TOPIC to BGR images and publishes them under PUBLISH_TOPIC
 * @version 0.1
 * @date 2021-11-21
 * 
 * @copyright Copyright (c) 2021
 * 
 */


// Include the ROS library
#include <ros/ros.h>

// Include opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Topics
#define IMAGE_TOPIC "/xtion/rgb/image_raw"
#define PUBLISH_TOPIC "/image_converter/output"

// Publisher
ros::Publisher pub;

void image_cb(const sensor_msgs::ImageConstPtr &msg)
{
  std_msgs::Header msg_header = msg->header;
  std::string frame_id = msg_header.frame_id.c_str();

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat image;
  cv::cvtColor(cv_ptr->image, image, cv::COLOR_RGB2BGR);


  sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  pub.publish(img_msg);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "image_converter");
  ros::NodeHandle n;

  pub = n.advertise<sensor_msgs::Image>(PUBLISH_TOPIC, 2);
  ros::Subscriber sub = n.subscribe(IMAGE_TOPIC, 1000, image_cb);

  ROS_INFO("Converting all incoming images...");

  ros::spin();

  return 0;
}