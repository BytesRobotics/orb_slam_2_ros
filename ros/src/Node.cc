#include "Node.h"

#include <iostream>

Node::Node (ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport): transform_listener(tfBuffer) {
  name_of_node_ = ros::this_node::getName();
  node_handle_ = node_handle;
  min_observations_per_point_ = 2;

  //static parameters
  node_handle_.param(name_of_node_+ "/publish_pointcloud", publish_pointcloud_param_, true);
  node_handle_.param(name_of_node_+ "/publish_pose", publish_pose_param_, true);
  node_handle_.param(name_of_node_+ "/publish_tf", publish_tf_param_, true);
  node_handle_.param<std::string>(name_of_node_+ "/map_frame_id", map_frame_id_param_, "map");
  node_handle_.param<std::string>(name_of_node_+ "/camera_frame_id", camera_frame_id_param_, "camera_link");
  node_handle_.param<std::string>(name_of_node_+ "/base_frame_id", base_frame_id_param_, "base_link");
  node_handle_.param<std::string>(name_of_node_ + "/map_file", map_file_name_param_, "map.bin");
  node_handle_.param<std::string>(name_of_node_ + "/voc_file", voc_file_name_param_, "file_not_set");
  node_handle_.param<std::string>(name_of_node_ + "/settings_file", settings_file_name_param_, "file_not_set");
  node_handle_.param(name_of_node_ + "/load_map", load_map_param_, false);

  orb_slam_ = new ORB_SLAM2::System (voc_file_name_param_, sensor, node_handle_, map_file_name_param_, load_map_param_);

  service_server_ = node_handle_.advertiseService(name_of_node_+"/save_map", &Node::SaveMapSrv, this);

  //Setup dynamic reconfigure
  dynamic_reconfigure::Server<orb_slam2_ros::dynamic_reconfigureConfig>::CallbackType dynamic_param_callback;
  dynamic_param_callback = boost::bind(&Node::ParamsChangedCallback, this, _1, _2);
  dynamic_param_server_.setCallback(dynamic_param_callback);

  //Set the zero point for the map so that we can incorporate odom correctly
  //camera_frame_id_param_ = frame_id   :   base_frame_id_param_ = child_frame_id
  geometry_msgs::TransformStamped base_to_camera_zero_msg = tfBuffer.lookupTransform(base_frame_id_param_, camera_frame_id_param_, ros::Time(0));
  tf2::fromMsg(base_to_camera_zero_msg, base_to_camera_zero);

  rendered_image_publisher_ = image_transport.advertise (name_of_node_+"/debug_image", 1);
  if (publish_pointcloud_param_) {
    map_points_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2> (name_of_node_+"/map_points", 1);
  }

  // Enable publishing camera's pose as PoseStamped message
  if (publish_pose_param_) {
    pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped> (name_of_node_+"/pose", 1);
  }

}


Node::~Node () {
  // Stop all threads
  orb_slam_->Shutdown();

  // Save camera trajectory
  orb_slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  delete orb_slam_;
}


void Node::Update () {
  cv::Mat position = orb_slam_->GetCurrentPosition();

  if (!position.empty()) {
      if(publish_tf_param_) {
          PublishPositionAsTransform(position);
      }

      if (publish_pose_param_) {
          PublishPositionAsPoseStamped (position);
      }
  }

  PublishRenderedImage (orb_slam_->DrawCurrentFrame());

  if (publish_pointcloud_param_) {
    PublishMapPoints (orb_slam_->GetAllMapPoints());
  }

}


void Node::PublishMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points) {
  sensor_msgs::PointCloud2 cloud = MapPointsToPointCloud (map_points);
  map_points_publisher_.publish (cloud);
}


void Node::PublishPositionAsTransform (cv::Mat position) {
  tf2::Transform map_to_camera = TransformFromMat (position);

  // Compute the appropriate transform to the base frame (usually odom or base_link)
  geometry_msgs::TransformStamped base_to_camera_msg = tfBuffer.lookupTransform(base_frame_id_param_, camera_frame_id_param_, ros::Time(0));

  //convert base transform to tf2::transform from geometry_msg
  tf2::Stamped<tf2::Transform> base_to_camera;
  tf2::fromMsg(base_to_camera_msg, base_to_camera);

  // Handle zero on the position to shift odom into correct frame with VO
  base_to_camera *= base_to_camera_zero.inverse(); // base_to_camera - base_to_camera_zero

  //base_to_camera.inverseTimes(base_to_camera_zero); //base_to_camera -> base_to_camera_zero
  tf2::Transform map_to_base = map_to_camera * base_to_camera.inverse(); // map_to_camera - base_to_camera

  //Print TF
//  geometry_msgs::PoseStamped print_msg;
//  tf2::toMsg(base_to_camera, print_msg);
//  cout << print_msg;
  // ###

  //Print TF
//  tf2::toMsg(tf2::Stamped<tf2::Transform>(map_to_camera, current_frame_time_, map_frame_id_param_), print_msg);
//  cout << print_msg;
  // ###

  //Print TF
//  tf2::toMsg(tf2::Stamped<tf2::Transform>(map_to_base, current_frame_time_, map_frame_id_param_), print_msg);
//  cout << print_msg;
  // ###

  // This is the transform that goes from /map to /odom knowing that image stems from camera link
  geometry_msgs::TransformStamped target_transform = tf2::toMsg(tf2::Stamped<tf2::Transform>(map_to_base, current_frame_time_, map_frame_id_param_));
  target_transform.child_frame_id = base_frame_id_param_;
  br.sendTransform(target_transform);
}

void Node::PublishPositionAsPoseStamped (cv::Mat position) {
  tf2::Transform grasp_tf = TransformFromMat (position);
  geometry_msgs::PoseStamped pose_msg;
  tf2::toMsg(tf2::Stamped<tf2::Transform>(grasp_tf, current_frame_time_, map_frame_id_param_), pose_msg);
  pose_publisher_.publish(pose_msg);
}


void Node::PublishRenderedImage (cv::Mat image) {
  std_msgs::Header header;
  header.stamp = current_frame_time_;
  header.frame_id = map_frame_id_param_;
  const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  rendered_image_publisher_.publish(rendered_image_msg);
}


tf2::Transform Node::TransformFromMat (cv::Mat position_mat) {
  cv::Mat rotation(3,3,CV_32F);
  cv::Mat translation(3,1,CV_32F);

  rotation = position_mat.rowRange(0,3).colRange(0,3);
  translation = position_mat.rowRange(0,3).col(3);


  tf2::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
                                    rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
                                    rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2)
                                   );

  tf2::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));

  //Coordinate transformation matrix from orb coordinate system to ros coordinate system
  const tf2::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                    -1, 0, 0,
                                     0,-1, 0);

  //Transform from orb coordinate system to ros coordinate system on camera coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  //Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

  //Transform from orb coordinate system to ros coordinate system on map coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  return tf2::Transform (tf_camera_rotation, tf_camera_translation);
}


sensor_msgs::PointCloud2 Node::MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> map_points) {
  if (map_points.size() == 0) {
    std::cout << "Map point vector is empty!" << std::endl;
  }

  sensor_msgs::PointCloud2 cloud;

  const int num_channels = 3; // x y z

  cloud.header.stamp = current_frame_time_;
  cloud.header.frame_id = map_frame_id_param_;
  cloud.height = 1;
  cloud.width = map_points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = { "x", "y", "z"};
  for (int i = 0; i<num_channels; i++) {
  	cloud.fields[i].name = channel_id[i];
  	cloud.fields[i].offset = i * sizeof(float);
  	cloud.fields[i].count = 1;
  	cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

	unsigned char *cloud_data_ptr = &(cloud.data[0]);

  float data_array[num_channels];
  for (unsigned int i=0; i<cloud.width; i++) {
    if (map_points.at(i)->nObs >= min_observations_per_point_) {
      data_array[0] = map_points.at(i)->GetWorldPos().at<float> (2); //x. Do the transformation by just reading at the position of z instead of x
      data_array[1] = -1.0* map_points.at(i)->GetWorldPos().at<float> (0); //y. Do the transformation by just reading at the position of x instead of y
      data_array[2] = -1.0* map_points.at(i)->GetWorldPos().at<float> (1); //z. Do the transformation by just reading at the position of y instead of z
      //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

      memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
    }
  }

  return cloud;
}


void Node::ParamsChangedCallback(orb_slam2_ros::dynamic_reconfigureConfig &config, uint32_t level) {
  orb_slam_->EnableLocalizationOnly (config.localize_only);
  min_observations_per_point_ = config.min_observations_for_ros_map;

  if (config.reset_map) {
    orb_slam_->Reset();
    config.reset_map = false;
  }

  orb_slam_->SetMinimumKeyFrames (config.min_num_kf_in_map);
}


bool Node::SaveMapSrv (orb_slam2_ros::SaveMap::Request &req, orb_slam2_ros::SaveMap::Response &res) {
  res.success = orb_slam_->SaveMap(req.name);

  if (res.success) {
    ROS_INFO_STREAM ("Map was saved as " << req.name);
  } else {
    ROS_ERROR ("Map could not be saved.");
  }

  return res.success;
}