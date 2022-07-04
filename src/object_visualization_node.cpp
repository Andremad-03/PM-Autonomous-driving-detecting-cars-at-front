#include "object_visualization_node.h"
#include "pointcloud_utils.h"



void cb_camera_info(const sensor_msgs::CameraInfo::ConstPtr &msg){

  if(!no_camera_info) return;
  no_camera_info = false;
  cam_model.fromCameraInfo(msg);

}

cv::Point3f find_centroid(const PointCloud_color::Ptr pointcloud){


  int x = 0, y = 0, z= 0;
  int size = pointcloud->size();
  cv::Point3d centroid;
  int cont = 0;
  for (int i = 0; i < size; i++ )
  {

      x += pointcloud->at(i).x;
      y += pointcloud->at(i).y;
      z += pointcloud->at(i).z;
  }

  centroid.x = x / size;
  centroid.y = y / size;
  centroid.z = z / size;

  return centroid;
}





std::string getcords(int x, int y, int z){

  std::string cords;

  cords = "XYZ = (" + std::to_string(x) +"," + std::to_string(y) +"," + std::to_string(z) +")";

  return cords;
}


void warning_visualization(const cv::Mat original_image){

  if(!depth_map_ready) return;

  cv::Point3f centroid;

  warning_image = original_image.clone();


  for (size_t i = 0; i< objects_detected.size();++i) {

    cv::Point vertex1(objects_detected.at(i).xmin ,objects_detected.at(i).ymin);
    cv::Point vertex2(objects_detected.at(i).xmax ,objects_detected.at(i).ymax);

    //pointcloud_camera tá vazia
     std::cout << "pointcloud_camera size:  " << pointcloud_camera->size() << std::endl;


    pcl_ros::transformPointCloud<pcl::PointXYZRGB>(*pointcloud_camera,*pointcloud_camera_base,transform);

    sensor_msgs::PointCloud2 pc_test_base;
    pointcloud_camera_base->header.frame_id = "base_link";
    pcl::toROSMsg(*pointcloud_camera_base,pc_test_base);


    pub_testing_pointcloud_base.publish(pc_test_base);

    centroid = find_centroid(pointcloud_camera_base);

    //Se perigoso
    if(centroid.x < 10 or abs(centroid.y) < 5){
            cv::rectangle(warning_image,vertex1,vertex2,red_color,2);
    } else{
      //se não perigoso
      cv::rectangle(warning_image,vertex1,vertex2,green_color,2);
    }


    cv::putText(warning_image,
                getcords(centroid.x,centroid.y,centroid.z),
                cv::Point(vertex1.x ,vertex1.y-10),cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(255,255,255));


  }
//  std::cout << "print imageine "<< std::endl;

  cv::imshow("warning sign",warning_image);
  cv::waitKey(10);


  sensor_msgs::ImagePtr output_image = cv_bridge::CvImage(std_msgs::Header(), "16UC1", warning_image).toImageMsg();

  pub_warning_image.publish(output_image);

  depth_map_ready =false;

}


void cb_image(const sensor_msgs::Image::ConstPtr &msg){
  seq_image = msg->header.seq;

  cv::Mat curr_image = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
  image_map.insert(std::make_pair(seq_image, curr_image));

  //elimino todas as imagens que não tiverem seq menor que o seq detect pq não vao ser usadas
  for (auto it = image_map.cbegin(); it != image_map.end() /* not hoisted */; /* no increment */)
  {
    if(it->first < seq_detect){
      image_map.erase(it++);    // or "it = m.erase(it)" since C++11
    }else{
      ++it;
    }
  }



}


void cb_detections(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){

  seq_detect = msg->image_header.seq;

  //se não houver uma imagem para aquele seq ele espera
  if(image_map.count(seq_detect) == 0) return;


  objects_detected.clear();
  objects_detected.resize(msg->bounding_boxes.size());

  for (darknet_ros_msgs::BoundingBox bb_box : msg->bounding_boxes) {
    objects_detected.push_back(bb_box);
  }

  warning_visualization(image_map.at(seq_detect)  );
}


void cb_depth_map(const sensor_msgs::Image::ConstPtr &msg){
  if(no_camera_info) return;
  if(image_map.count(seq_detect) == 0) return;

  std::vector<cv::KeyPoint> keypoints;


  depth_map = cv_bridge::toCvShare(msg, "16UC1")->image.clone();


  pointcloud_camera->clear();
  depthmap_to_pointcloud(depth_map, keypoints, image_map.at(seq_detect), cam_model, pointcloud_camera );


  sensor_msgs::PointCloud2 pc_test_vel;
  pointcloud_camera->header.frame_id = "vision_frame";
  pcl::toROSMsg(*pointcloud_camera,pc_test_vel);


  pub_testing_pointcloud_vel.publish(pc_test_vel);


  depth_map_ready = true;

}





int main(int argc, char **argv)
{


  ros::init(argc, argv, "object_visualization");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_img = it.subscribe("stereo/left/image_rect_color",1, cb_image);

  ros::Subscriber sub_detections = nh.subscribe("/objects/left/bounding_boxes",1,cb_detections);
  ros::Subscriber sub_depth_map = nh.subscribe("/depth_map",1,cb_depth_map);
  ros::Subscriber sub_camera_info = nh.subscribe("/stereo/left/camera_info",1,cb_camera_info);


  pointcloud_camera_base.reset(new PointCloud_color);
  pointcloud_camera.reset(new PointCloud_color);

  pub_warning_image = nh.advertise<sensor_msgs::Image>("/warning_sign",1);
  pub_testing_pointcloud_vel = nh.advertise<PointCloud_color>("/test/pointcloud/vel",1);
  pub_testing_pointcloud_base = nh.advertise<PointCloud_color>("/test/pointcloud/base",1);


  tf_listener = new tf::TransformListener();

  try {
    //Converting from velodyne to base_link
      tf_listener->waitForTransform("/base_link","/velodyne",ros::Time::now(),ros::Duration(3.0));

      tf_listener->lookupTransform("/base_link", "/vision_frame",ros::Time::now(),transform);

  } catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }




  ros::spin();

  return 0;
}
