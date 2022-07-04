#ifndef OBJECT_3D_ESTIMATION_NODE_H
#define OBJECT_3D_ESTIMATION_NODE_H

//ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/tf.h>


#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/opencv.hpp>

#include <pcl_ros/point_cloud.h>

#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/transforms.h>

#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


#include <darknet_ros_msgs/BoundingBoxes.h>


#include <image_geometry/pinhole_camera_model.h>

#define HISTERESE 1E-5f

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud_color;

bool no_camera_info = true;
bool image_ready, pointcloud_ready, detection_ready;


PointCloud_color::Ptr msg_cloud_color;

cv::Mat depth_map;

ros::Publisher pub_cloud_color;


cv::Mat left_image;
std::vector<darknet_ros_msgs::BoundingBox> objects_detected;
sensor_msgs::CameraInfo camera_info;
image_geometry::PinholeCameraModel cam_model;

std::string frame_name;

ros::Publisher pub_pointcloud_camera,pub_test2, pub_depth_map;
PointCloud_color pc_velodyne,pc_velodyne_transformed;
PointCloud velodyne_cloud, velodyne_cloud_transform;

PointCloud_color::Ptr msg_cloud;
PointCloud_color::Ptr transf_filt,transf_normal;
PointCloud_color::Ptr filtered_cam_cloud;

double min_x=1000, min_y=1000, max_z;

//bool flag_image, flag_pointcloud, flag_detection;

bool flag_left = false;
bool pointcloud_color=true;

double gain = 1.0;

tf::TransformListener *tf_listener;


double fov_x,fov_y, cx, cy, fx, fy;
int width, height;

cv::Ptr<cv::Feature2D> featureDetector = cv::ORB::create();
std::vector<cv::KeyPoint> keypoints_l, keypoints_ROI;
cv::Mat descriptors_l, descriptors_ROI;


int seq_image = -1, seq_detect = -2;
std::map<int,cv::Mat> image_map;


bool flag_first = true;



double calculate_distance(double x, double y, double z){
  return sqrt(x*x +y*y + z*z);
}

void cb_camera_info(const sensor_msgs::CameraInfo::ConstPtr &msg);
void cb_rect_color_image(const sensor_msgs::Image::ConstPtr& msg);
void cb_pointcloud_velodyne(const sensor_msgs::PointCloud2::ConstPtr& msg);
void cb_darknet_detections(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
void cb_test(const sensor_msgs::Image::ConstPtr& msg);
void filter_pointcloud(const PointCloud_color::Ptr raw_pointcloud, PointCloud_color::Ptr &filtered_pointcloud);
void get_depth_map(const PointCloud_color::Ptr pointcloud, cv::Mat &depth_map);
void pointcloud_operations();




#endif // OBJECT_3D_ESTIMATION_NODE_H
