// Copyright (c) 2018 Intel Corporation. All Rights Reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GRASP_LIBRARY__ROS2__GRASP_DETECTOR_GPD_HPP_
#define GRASP_LIBRARY__ROS2__GRASP_DETECTOR_GPD_HPP_

// ROS2
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#ifdef RECOGNIZE_PICK
#include <people_msgs/msg/objects_in_masks.hpp>
#endif
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// PCL
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// eigen
#include <Eigen/Geometry>

// GPG
#include <gpd/util/cloud.h>

// GPD
#include <gpd/candidate/hand.h>
#include <gpd/candidate/hand_geometry.h>
#include <gpd/candidate/hand_search.h>
#include <gpd/grasp_detector.h>
#include <grasp_msgs/msg/grasp_config.hpp>
#include <grasp_msgs/msg/grasp_config_list.hpp>

// system
#include <algorithm>
#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "grasp_library/ros2/consts.hpp"
#include "grasp_library/ros2/grasp_detector_base.hpp"
#include "grasp_library/ros2/grasp_planner.hpp"

namespace grasp_ros2
{

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudPointNormal;


/** GraspDetectorGPD class
 *
 * \brief A ROS node that can detect grasp poses in a point cloud.
 *
 * This class is a ROS node that handles all the ROS topics.
 *
*/
class GraspDetectorGPD : public rclcpp::Node, public GraspDetectorBase
{
public:
  /**
   * \brief Constructor.
  */
  // Constructor with config file
  explicit GraspDetectorGPD(const std::string& config_file, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  
  // Constructor with just node options (for component loading)
  explicit GraspDetectorGPD(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * \brief Destructor.
  */
  ~GraspDetectorGPD()
  {
    delete cloud_camera_;

    // todo stop and delete threads
  }

private:
  /**
   * \brief Run the ROS node. Loops while waiting for incoming ROS messages.
   */
  void onInit();

  /**
   * \brief Detect grasp poses in a point cloud received from a ROS topic.
   * \return the list of grasp poses
   */
  std::vector<std::unique_ptr<gpd::candidate::Hand>> detectGraspPosesInTopic();

  /**
   * \brief Callback function for the ROS topic that contains the input point cloud.
   * \param msg the incoming ROS message
   */
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
#ifdef RECOGNIZE_PICK
  /**
   * \brief Callback function for the ROS topic that contains the detected and segmented objects
   * \param msg The detected objects message
   */
  void object_callback(const people_msgs::msg::ObjectsInMasks::SharedPtr msg);
#endif
  /**
   * \brief Create a ROS message that contains a list of grasp poses from a list of handles.
   * \param hands the list of grasps
   * \return the ROS message that contains the grasp poses
   */
  grasp_msgs::msg::GraspConfigList createGraspListMsg(const std::vector<std::unique_ptr<gpd::candidate::Hand>> & hands);
  
  // Backward compatibility method for GPG style grasps
  grasp_msgs::msg::GraspConfigList createGraspListMsg(const std::vector<gpd::candidate::Hand> & hands);

  /**
   * \brief Convert GPD Grasp into grasp message.
   * \param hand A GPD grasp
   * \return The Grasp message converted
   */
  grasp_msgs::msg::GraspConfig convertToGraspMsg(const gpd::candidate::Hand & hand);

  /**
   * \brief Convert GPD Grasps into visual grasp messages.
   */
  visualization_msgs::msg::MarkerArray convertToVisualGraspMsg(
    const std::vector<std::unique_ptr<gpd::candidate::Hand>> & hands,
    double outer_diameter, double hand_depth, double finger_width, double hand_height,
    const std::string & frame_id);
    
  // Backward compatibility method for GPG style grasps
  visualization_msgs::msg::MarkerArray convertToVisualGraspMsg(
    const std::vector<gpd::candidate::Hand> & hands,
    double outer_diameter, double hand_depth, double finger_width, double hand_height,
    const std::string & frame_id);

  /**
   * \brief Create finger marker for visual grasp messages
   */
  visualization_msgs::msg::Marker createFingerMarker(
    const Eigen::Vector3d & center,
    const Eigen::Matrix3d & frame, double length, double width, double height, int id,
    const std::string & frame_id);

  /**
   * \brief Create hand base marker for visual grasp messages
   */
  visualization_msgs::msg::Marker createHandBaseMarker(
    const Eigen::Vector3d & start,
    const Eigen::Vector3d & end, const Eigen::Matrix3d & frame, double length, double height,
    int id,
    const std::string & frame_id);

  /** Converts an Eigen Vector into a Point message. Todo ROS2 eigen_conversions*/
  void pointEigenToMsg(const Eigen::Vector3d & e, geometry_msgs::msg::Point & m)
  {
    m.x = e(0);
    m.y = e(1);
    m.z = e(2);
  }

  /** Converts an Eigen Vector into a Vector message. Todo ROS2 eigen_conversions*/
  void vectorEigenToMsg(const Eigen::Vector3d & e, geometry_msgs::msg::Vector3 & m)
  {
    m.x = e(0);
    m.y = e(1);
    m.z = e(2);
  }

  /**
   * \brief Initialize ROS components such as publishers, subscribers, etc.
   */
  void initializeROSComponents();

  Eigen::Vector3d view_point_; /**< (input) view point of the camera onto the point cloud*/
  /** individual parameters for grasp detection */
  /** stores point cloud with (optional) camera information and surface normals*/
  gpd::util::Cloud * cloud_camera_;
  std_msgs::msg::Header cloud_camera_header_; /**< stores header of the point cloud*/
  /** status variables for received (input) messages*/
  bool has_cloud_;
  std::string frame_; /**< point cloud frame*/
  bool auto_mode_; /**< grasp detection mode*/
  bool plane_remove_; /**< whether enable object detection>*/
#ifdef RECOGNIZE_PICK
  /** the latest message on detected objects*/
  people_msgs::msg::ObjectsInMasks::SharedPtr object_msg_;
#endif
  std::vector<double> grasp_ws_;

  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber2_;
  /** ROS2 subscriber for point cloud messages*/
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
#ifdef RECOGNIZE_PICK
  /** ROS2 subscriber for object  messages*/
  rclcpp::Subscription<people_msgs::msg::ObjectsInMasks>::SharedPtr object_sub_;
#endif
  /** ROS2 publisher for grasp list messages*/
  rclcpp::Publisher<grasp_msgs::msg::GraspConfigList>::SharedPtr grasps_pub_;
  /** ROS2 publisher for filtered point clouds*/
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
  /** ROS2 publisher for grasps in rviz (visualization)*/
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grasps_rviz_pub_;

  std::unique_ptr<gpd::GraspDetector> grasp_detector_; /**< used to run the grasp pose detection*/
  gpd::candidate::HandGeometry hand_geometry_; /**< hand geometry parameters */
  std::string config_file_; /**< path to the GPD config file */
  rclcpp::Logger logger_ = rclcpp::get_logger("GraspDetectorGPD");
  std::thread * detector_thread_; /**< thread for grasp detection*/
  
  /**
   * \brief Check if the detector is properly initialized
   * \return true if detector is valid, false otherwise
   */
  bool isDetectorValid() const;
};

}  // namespace grasp_ros2

#endif  // GRASP_LIBRARY__ROS2__GRASP_DETECTOR_GPD_HPP_
