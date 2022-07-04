#ifndef POINTCLOUD_UTILS_H
#define POINTCLOUD_UTILS_H

//#include "object_3d_estimation_node.h"

#include <pcl_ros/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud_color;


double intensity_to_depth(double x, double y, double maxIntensity, double gain, double intensity){

  double depth;

  depth = intensity;
  depth = depth - maxIntensity;
  depth = -depth/gain;
  depth = depth * depth;
  depth = depth - x*x - y*y;
  depth = sqrt(depth); //z

  return depth;

}


void depthmap_to_pointcloud(const cv::Mat depth_map_to_convert, std::vector<cv::KeyPoint> keypoints, cv::Mat image,
                            image_geometry::PinholeCameraModel cam, PointCloud_color::Ptr & pointcloud ){

  bool use_keypoints = true;
  if(keypoints.size() == 0){
    use_keypoints = false;
  }


  cv::Point2d pt;
  cv::Point3d pt_xyz;
  pcl::PointXYZRGB pcl_color_point;


  for(int row = 0; row < depth_map_to_convert.rows ; row++){
    for (int col = 0; col < depth_map_to_convert.cols; col++){

      if(use_keypoints){
        for (int i = 0; i < keypoints.size(); ++i) {
          if(row != keypoints[i].pt.y or col != keypoints[i].pt.x)
            continue;
        }
      }

      pt.x = col;
      pt.y = row;

      pt_xyz = cam.projectPixelTo3dRay(pt);


      pcl_color_point.x = pt_xyz.x;
      pcl_color_point.y = pt_xyz.y;

      pcl_color_point.z =  intensity_to_depth(pt_xyz.x, pt_xyz.y, 255, 10, depth_map_to_convert.at<double>(row,col));

      pcl_color_point.r = image.at<cv::Vec3b>(pt)[2];
      pcl_color_point.g = image.at<cv::Vec3b>(pt)[1];
      pcl_color_point.b = image.at<cv::Vec3b>(pt)[0];


      pointcloud->push_back(pcl_color_point);

    }
  }

}

#endif // POINTCLOUD_UTILS_H
