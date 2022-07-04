#include "object_3d_estimation_node.h"
#include "pointcloud_utils.h"

using namespace std;
using namespace cv;

//We need to find fov of the camera to filter the image
void cb_camera_info(const sensor_msgs::CameraInfo::ConstPtr &msg){

  if(!no_camera_info) return;
  no_camera_info = false;
  cam_model.fromCameraInfo(msg);


  width = msg-> width;
  height = msg->height;


  fx = msg->K[0];
  fy = msg->K[4];
  cx = msg->K[2];
  cy = msg->K[5];

  Point2d principal_point;
  double aspect_ratio,focal_length;
  sensor_msgs::CameraInfo::_K_type intrinsic_matrix = msg->K;

  Mat intrk(3,3,CV_64FC1, (void *) msg->K.data());

  calibrationMatrixValues(intrk,Size(msg->width,msg->height),
                                msg->width,msg->height,
                                fov_x,fov_y,focal_length,
                                principal_point,aspect_ratio);


  fov_x = fov_x/ 180*CV_PI;
  fov_y = fov_y/ 180*CV_PI;


}


int find_closest_car_index(const std::vector<darknet_ros_msgs::BoundingBox> bboxes){

  int closest_car_index = -1;
  int max_bbox_area = -1;
  int bbox_area;
  int width_roi, height_roi;
  string car_class = "car";

  //Closest Car
  for(size_t i = 0; i < bboxes.size(); i++){

    if (car_class.compare(bboxes[i].Class) == 0){


      width_roi = bboxes[i].xmax - bboxes[i].xmin;
      height_roi = bboxes[i].ymax - bboxes[i].ymin;


      bbox_area = width_roi*height_roi;
      if(bbox_area > max_bbox_area){
        max_bbox_area = bbox_area;
        closest_car_index = i;
      }
    }
  }
  return closest_car_index;
}


void ColorAnd3D_car(std::vector<darknet_ros_msgs::BoundingBox> bbox, cv::Mat original_image){

  std::vector<cv::Point2f> feat_ref, feat_ROI;
  pcl::PointXYZRGB pcl_color_point;


  int closest_car_index = find_closest_car_index(bbox);
  if(closest_car_index == -1){
    cout << "Can't find car" << endl;
    return;
  }

  std::cout << "max i: " << closest_car_index << "\n" << std::endl;


  // Cropping car image
  cv::Rect crop_region(bbox[closest_car_index].xmin, bbox[closest_car_index].ymin,
                       bbox[closest_car_index].xmax-bbox[closest_car_index].xmin,
                       bbox[closest_car_index].ymax-bbox[closest_car_index].ymin);
  //Imagem do carro
  cv::Mat ROI = original_image(crop_region);
//    if (flag_first){
//    cv:imshow("ROI", ROI);
//    char key = waitKey(0);
//    if(key == 27){
//      cv::destroyAllWindows();
//    }
//    flag_first = false;
//  }

  //Detect features
  featureDetector->detect(ROI,keypoints_ROI);
  cv::KeyPoint::convert(keypoints_ROI,feat_ROI);
//  std::cout << "keypoints size: " << keypoints_ROI.size() << "\n" << std::endl;


  //Depth map para xyz
  msg_cloud_color->clear();
  depthmap_to_pointcloud(depth_map,  keypoints_ROI, original_image,cam_model ,msg_cloud_color);


  //Escrevendo o stamp do ros no header da mensagem
  pcl_conversions::toPCL(ros::Time::now(),msg_cloud_color->header.stamp);


  //Colocando o header(frame_id) da mesma forma que se recebe a informação
  msg_cloud_color->header.frame_id = "vision_frame";
  pub_cloud_color.publish(msg_cloud_color);
}


void cb_rect_color_image(const sensor_msgs::Image::ConstPtr& msg){

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

//  cout << "cb_rect_color_image callback"<<endl;

  image_ready = true;
//  left_image = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
//  flag_left = true;

//  cv::resize(left_image, left_image, Size(left_image.cols/2,left_image.rows/2));

  //imshow("left",left_image);
  //waitKey(0);
}


void cb_pointcloud_velodyne(const sensor_msgs::PointCloud2::ConstPtr& msg){

//  cout << "Velodyne callback"<<endl;
  pointcloud_ready = true;

  pcl::fromROSMsg(*msg,pc_velodyne); //Vai dar "Failed to find match for field 'rgb'." mas também não sei como se resolve


  //Publicar para ver se está a dar
  sensor_msgs::PointCloud2 test;
  pcl::toROSMsg(pc_velodyne,test);


  pub_test2.publish(test);



  pointcloud_operations();


}


void cb_darknet_detections(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){

  seq_detect = msg->image_header.seq;

  //se não houver uma imagem para aquele seq ele espera
  if(image_map.count(seq_detect) == 0) return;


  detection_ready = true;
  objects_detected.clear();
  objects_detected.resize(msg->bounding_boxes.size());

  for (darknet_ros_msgs::BoundingBox bb_box : msg->bounding_boxes) {

    objects_detected.push_back(bb_box);

  }

  ColorAnd3D_car(objects_detected,image_map.at(seq_detect));



}



//Filters and limits the point cloud to camera's fov
void filter_pointcloud(const PointCloud_color::Ptr raw_pointcloud, PointCloud_color::Ptr &filtered_pointcloud){


//  cout << "filter cloud" <<endl;

  PointCloud_color::Ptr reduced_depth_pointcloud (new PointCloud_color ());


  //Limiting max depth
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (raw_pointcloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 50.0);
  pass.filter (*reduced_depth_pointcloud);


  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_circles (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_circles (new pcl::PointIndices);




  // Estimate point normals
   ne.setSearchMethod (tree);
   ne.setInputCloud (reduced_depth_pointcloud);
   ne.setKSearch (50);
   ne.compute (*cloud_normals);

   // Create the segmentation object for the planar model and set all the parameters
   seg.setOptimizeCoefficients (true);
   seg.setModelType (pcl::SACMODEL_CIRCLE2D);
   seg.setNormalDistanceWeight (1);
   seg.setMethodType (pcl::SAC_RANSAC);
   seg.setMaxIterations (1500);
   seg.setDistanceThreshold (0.2);
   seg.setRadiusLimits (0, 1000);
   seg.setInputCloud (reduced_depth_pointcloud);
   seg.setInputNormals (cloud_normals);

   // Obtain the plane inliers and coefficients
   seg.segment (*inliers_circles, *coefficients_circles);


   // Extract the planar inliers from the input cloud
    extract.setInputCloud (reduced_depth_pointcloud);
    extract.setIndices (inliers_circles);
    extract.setNegative (true); // true -> removes the circles from the image; false -> only the circles appear

    PointCloud_color::Ptr cloud_no_fov (new PointCloud_color ());

    //Removing the circles and storing in another pointcloud
    extract.filter (*cloud_no_fov);

    //Limiting the point cloud
    for(PointCloud_color::iterator pc_it = cloud_no_fov->points.begin(); pc_it < cloud_no_fov->points.end(); ++pc_it){
      //cout << *pc_it <<endl;

      //ignores the points that are behind the camera
      if(pc_it->z < 0) continue;

      //Only stores points belonging to camera's fov
      if(abs(pc_it->x / (pc_it->z+HISTERESE)) < tan(fov_y)   and
         abs(pc_it->y / (pc_it->z+HISTERESE)) < tan(fov_x)){
            filtered_pointcloud->push_back(*pc_it);

            if(pc_it->x < min_x) min_x = pc_it->x;
            if(pc_it->y < min_y) min_y = pc_it->y;
      }
    }


}





void get_depth_map(const PointCloud_color::Ptr pointcloud, Mat &depth_map){
  //cout << "depth_map" << endl;
  int cont = 0;

  double gain = 1.0;
  double z,u,v;
  double distance;
  int pixel_pos_x, pixel_pos_y;

  //cout <<  "mapsize: "  << depth_map.size()  << endl;

  for(PointCloud_color::iterator pc_it = pointcloud->points.begin(); pc_it < pointcloud->points.end(); ++pc_it){

    distance = calculate_distance(pc_it->x, pc_it->y,pc_it->z);

    cv::Point3f pt_cv(pc_it->x, pc_it->y, pc_it->z);//init Point3f
    cv::Point2f uv = cam_model.project3dToPixel(pt_cv); // project 3d point to 2d point


    pixel_pos_x = (int)round(uv.x);
    pixel_pos_y = (int)round(uv.y);

    depth_map.at<double>(pixel_pos_y,pixel_pos_x) = (255-std::min((int)distance*10,254));


  }

  depth_map.convertTo(depth_map,CV_16UC1);

}


void pointcloud_operations(){

  if(no_camera_info) return;

  if(!image_ready or !pointcloud_ready)
    return;
  else{
    image_ready = false;
    pointcloud_ready = false;
  }


  //Converting coords to vision frame (frame_name = vision_frame)
  pcl_ros::transformPointCloud<pcl::PointXYZRGB>(frame_name,pc_velodyne,*transf_normal,*tf_listener);


  PointCloud_color::Ptr camera_pointcloud (new PointCloud_color ());

  //Filter pointcloud and limit to camera fov
  filter_pointcloud(transf_normal,camera_pointcloud);



  //Publish camera's pointcloud
  sensor_msgs::PointCloud2 pc_transformed_test;
  camera_pointcloud->header.frame_id = frame_name;
  pcl::toROSMsg(*camera_pointcloud,pc_transformed_test);

  pub_pointcloud_camera.publish(pc_transformed_test);


  depth_map = cv::Mat(height, width, CV_32FC1,cv::Scalar(std::numeric_limits<float>::max()));
  get_depth_map(camera_pointcloud,depth_map);

  sensor_msgs::ImagePtr output_image = cv_bridge::CvImage(std_msgs::Header(), "16UC1", depth_map).toImageMsg();

  pub_depth_map.publish(output_image);


}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_3d_estimation");

  ros::NodeHandle nh;
  ros::NodeHandle n_private("~");

  image_transport::ImageTransport it(nh);

  image_transport::Subscriber sub_img = it.subscribe("stereo/left/image_rect_color",1, cb_rect_color_image);

  ros::Subscriber sub_pointcloud = nh.subscribe("/velodyne_points",1,cb_pointcloud_velodyne);

  ros::Subscriber sub_dark_detections = nh.subscribe("/objects/left/bounding_boxes",1,cb_darknet_detections);

  ros::Subscriber sub_camera_info = nh.subscribe("/stereo/left/camera_info",1,cb_camera_info);

  pub_cloud_color = nh.advertise<PointCloud_color>("/stereo/pointcloud_color",1);
  msg_cloud_color.reset(new PointCloud_color);



  //image_transport::Subscriber sub_test = it.subscribe("objects/left/detection_image",1,cb_test);

  pub_pointcloud_camera = nh.advertise<sensor_msgs::PointCloud2>("/assign2/pointcloud_camera",1);
  pub_test2 = nh.advertise<sensor_msgs::PointCloud2>("/test/Topic2",1);
  pub_depth_map = nh.advertise<sensor_msgs::Image>("/depth_map",1);
  pub_cloud_color = nh.advertise<PointCloud_color>("/stereo/pointcloud_color",1);



  msg_cloud.reset(new PointCloud_color);
  transf_filt.reset(new PointCloud_color);
  transf_normal.reset(new PointCloud_color);
  filtered_cam_cloud.reset(new PointCloud_color);
  tf_listener = new tf::TransformListener();
  msg_cloud_color.reset(new PointCloud_color);

  n_private.param<string>("frame_name",frame_name,"vision_frame");



  try {
    //LIDAR DATA está no frame velodyne e nós queremos converter para o frame "vision_frame"
      tf_listener->waitForTransform(frame_name,"velodyne",ros::Time::now(),ros::Duration(3.0));

  } catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }



  ros::spin();

  delete tf_listener;

  return 0;
}
