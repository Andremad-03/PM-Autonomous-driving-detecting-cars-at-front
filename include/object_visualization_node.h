#ifndef OBJECT_VISUALIZATION_H
#define OBJECT_VISUALIZATION_H

#include <ros/ros.h>

#include <tf/tf.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

//#define car_probability_threshold 65

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud_color;



bool image_ready = false, detection_ready = false, warning_ready = false, no_camera_info = true;
image_geometry::PinholeCameraModel cam_model;

cv::Mat raw_image, warning_image;
cv::Mat depth_map;

std::vector<darknet_ros_msgs::BoundingBox> objects_detected;
PointCloud_color::Ptr pointcloud_camera, pointcloud_camera_base;



std::map<int,cv::Mat> image_map;


ros::Publisher pub_warning_image, pub_testing_pointcloud_vel,pub_testing_pointcloud_base;

cv::Scalar green_color (0, 255, 0);
cv::Scalar red_color (0, 0, 255);

int seq_detect = -1 , seq_image = -2;
bool next_image = true;

int first_seq_image = -100;
int first_seq_detect = -100;

tf::TransformListener *tf_listener;
tf::StampedTransform transform;

bool depth_map_ready = false;







#endif // OBJECT_VISUALIZATION_H
