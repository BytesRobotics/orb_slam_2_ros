//
// Created by michael on 3/31/20.
//

#ifndef SRC_FRAMEMANAGER_H
#define SRC_FRAMEMANAGER_H

#include<string>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/imgproc/types_c.h>

#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>

/* The following class exists to provide reliable transformations between the robots odometry  source
 * and the visual odometry that is computed in this node. This also provides the ability to easily impement improved
 * motion models for predicting the location of the next frame. The odometry that is to be fed in to the motion model
 * includes Odometry filtered with an absolutely references IMU, an Altimeter, and wheel odometry GPS is included
 * separately in a solver that aims to take in particularly noisy GPS where no single point can be relied on and
 * convert slowly refine a transformation from the GPS into the camera state which will eventually allow for
 * the local state to publish a GPS coordinate and keep the robot geo-references.
 * This class also performs functions for saving and loading this transform information along with maps saved by
 * the visual odometry.
 *
 * IMPORTANT: This library requires that it be initialized with Odometry fused yaw with position determined by
 * an absolutely oriented IMU (9-DOF) with an accurate magnetometer. This is used to reconciles the GPS coordinate frames
 * with that of the ORB tracker.
 * */

class FrameManager{
public:
    FrameManager(ros::NodeHandle &node_handle, std::string base_frame, std::string map_frame, std::string odom_frame,
            bool include_gps, bool publish_transforms);
    bool setInitialOdomFrame();
    cv::Mat getTwo(); // Gets the transform from the VO world frame to the robot odometry frame
    cv::Mat getTwc(); // Gets the transform from the camera frame to the VO world frame
    cv::Mat getToc(); // Gets the transform from the robot odometry frame to the camera frame (this can be used for motion model)
private:
    bool include_gps_;
    bool publish_transforms_;

    // We will be using tf2 to handle the key transformation between odom, SLAM world, and camera frame
    tf2_ros::Buffer tfBuffer;
//    tf2_ros::TransformListener transform_listener;
//    tf2_ros::TransformBroadcaster br;
    tf2::Stamped<tf2::Transform> base_to_camera_zero;
};

#endif //SRC_FRAMEMANAGER_H
