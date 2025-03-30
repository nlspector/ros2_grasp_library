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

#include <string>
#include <vector>

#include "grasp_library/ros2/ros_params.hpp"

namespace grasp_ros2
{

std::string ROSParameters::getDetectionParams(rclcpp::Node * node)
{
  // Create a temporary file to store the configuration
  char temp_dir[] = "/tmp/gpd_config_XXXXXX";
  char* dir_name = mkdtemp(temp_dir);
  if (dir_name == nullptr) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create temporary directory for GPD config");
    return "";
  }
  
  std::string config_file_path = std::string(dir_name) + "/gpd_config.yaml";
  std::ofstream config_file(config_file_path);
  
  if (!config_file.is_open()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create GPD config file at %s", config_file_path.c_str());
    return "";
  }
  
  // Read hand geometry parameters.
  double finger_width, hand_outer_diameter, hand_depth, hand_height, init_bite;
  node->get_parameter_or("finger_width", finger_width, 0.005);
  node->get_parameter_or("hand_outer_diameter", hand_outer_diameter, 0.12);
  node->get_parameter_or("hand_depth", hand_depth, 0.06);
  node->get_parameter_or("hand_height", hand_height, 0.02);
  node->get_parameter_or("init_bite", init_bite, 0.01);
  
  // Read local hand search parameters.
  double nn_radius;
  int num_orientations, num_samples, num_threads, rotation_axis;
  node->get_parameter_or("nn_radius", nn_radius, 0.01);
  node->get_parameter_or("num_orientations", num_orientations, 8);
  node->get_parameter_or("num_samples", num_samples, 100);
  node->get_parameter_or("num_threads", num_threads, 4);
  node->get_parameter_or("rotation_axis", rotation_axis, 2);
  
  // Read plotting parameters.
  bool plot_samples, plot_normals, plot_filtered_grasps, plot_valid_grasps;
  bool plot_clusters, plot_selected_grasps, plot_candidates;
  node->get_parameter_or("plot_samples", plot_samples, false);
  node->get_parameter_or("plot_normals", plot_normals, false);
  node->get_parameter_or("plot_filtered_grasps", plot_filtered_grasps, false);
  node->get_parameter_or("plot_valid_grasps", plot_valid_grasps, true);
  node->get_parameter_or("plot_clusters", plot_clusters, false);
  node->get_parameter_or("plot_selected_grasps", plot_selected_grasps, false);
  node->get_parameter_or("plot_candidates", plot_candidates, false);
  
  // Read preprocessing parameters.
  bool remove_outliers, voxelize;
  std::vector<double> workspace;
  node->get_parameter_or("remove_outliers", remove_outliers, false);
  node->get_parameter_or("voxelize", voxelize, true);
  node->get_parameter_or("workspace", workspace,
    std::vector<double>(std::initializer_list<double>({-1.0, 1.0, -1.0, 1.0, -1.0, 1.0})));
  
  // Read classification parameters and create classifier.
  std::string model_file, weights_file;
  double min_score_diff;
  bool create_image_batches;
  int device;
  node->get_parameter_or("trained_file", weights_file, std::string("/home/noahspector/ws_moveit/src/ros2_grasp_library/gpd/lenet/15channels/params/"));
  node->get_parameter_or("min_score_diff", min_score_diff, 500.0);
  node->get_parameter_or("create_image_batches", create_image_batches, false);
  node->get_parameter_or("device", device, 0);
  
  // Read grasp image parameters.
  int image_size, image_num_channels;
  node->get_parameter_or("image_size", image_size, 60);
  node->get_parameter_or("image_num_channels", image_num_channels, 15);
  
  // Read learning parameters.
  bool remove_plane;
  node->get_parameter_or("remove_plane_before_image_calculation", remove_plane, false);
  
  // Read grasp filtering parameters
  bool filter_grasps, filter_half_antipodal;
  node->get_parameter_or("filter_grasps", filter_grasps, false);
  node->get_parameter_or("filter_half_antipodal", filter_half_antipodal, false);
  std::vector<double> gripper_width_range = {0.03, 0.10};
  
  // Read clustering parameters
  int min_inliers, num_selected;
  node->get_parameter_or("min_inliers", min_inliers, 1);
  node->get_parameter_or("num_selected", num_selected, 5);
  
  // Write parameters to the config file in YAML format
  config_file << "# GPD Configuration File (Auto-generated)\n\n";
  
  // Hand geometry section
  config_file << "# Hand geometry parameters\n";
  config_file << "finger_width = " << finger_width << "  # finger width in meters\n";
  config_file << "hand_outer_diameter = " << hand_outer_diameter << "  # hand outer diameter in meters\n";
  config_file << "hand_depth = " << hand_depth << "  # hand depth in meters\n";
  config_file << "hand_height = " << hand_height << "  # hand height in meters\n";
  config_file << "init_bite = " << init_bite << "  # approach distance in meters\n\n";
  
  // Hand search parameters
  config_file << "# Local hand search parameters\n";
  config_file << "nn_radius = " << nn_radius << "  # radius for nearest neighbors search\n";
  config_file << "num_orientations = " << num_orientations << "  # number of hand orientations\n";
  config_file << "num_samples = " << num_samples << "  # number of samples\n";
  config_file << "num_threads = " << num_threads << "  # number of CPU threads\n";
  config_file << "rotation_axis = " << rotation_axis << "  # axis of rotation for hand orientations\n\n";
  
  // Plotting parameters
  config_file << "# Visualization parameters\n";
  config_file << "plot_samples = " << (plot_samples ? "true" : "false") << "\n";
  config_file << "plot_normals = " << (plot_normals ? "true" : "false") << "\n";
  config_file << "plot_filtered_grasps = " << (plot_filtered_grasps ? "true" : "false") << "\n";
  config_file << "plot_valid_grasps = " << (plot_valid_grasps ? "true" : "false") << "\n";
  config_file << "plot_clusters = " << (plot_clusters ? "true" : "false") << "\n";
  config_file << "plot_selected_grasps = " << (plot_selected_grasps ? "true" : "false") << "\n";
  config_file << "plot_candidates = " << (plot_candidates ? "true" : "false") << "\n\n";
  
  // Preprocessing parameters
  config_file << "# Preprocessing parameters\n";
  config_file << "remove_outliers = " << (remove_outliers ? "true" : "false") << "\n";
  config_file << "voxelize = " << (voxelize ? "true" : "false") << "\n";
  config_file << "workspace = [";
  for (size_t i =  0; i < workspace.size(); ++i) {
    config_file << workspace[i];
    if (i < workspace.size() - 1) {
      config_file << ", ";
    }
  }
  config_file << "]  # [minX, maxX, minY, maxY, minZ, maxZ]\n\n";
  
  // Classification parameters
  config_file << "# Classification parameters\n";
  config_file << "weights_file = " << weights_file << "  # path to weights file\n";
  config_file << "min_score_diff = " << min_score_diff << "  # minimum score difference\n";
  config_file << "create_image_batches = " << (create_image_batches ? "true" : "false") << "  # create image batches\n";
  config_file << "device = " << device << "  # device ID for GPU\n\n";
  
  // Image parameters
  config_file << "# Image parameters\n";
  config_file << "image_size = " << image_size << "  # size of the image (width and height)\n";
  config_file << "image_num_channels = " << image_num_channels << "  # number of image channels\n\n";
  
  // Filtering parameters
  config_file << "# Filtering parameters\n";
  config_file << "filter_grasps = " << (filter_grasps ? "true" : "false") << "\n";
  config_file << "filter_half_antipodal = " << (filter_half_antipodal ? "true" : "false") << "\n";
  config_file << "gripper_width_range = [" << gripper_width_range[0] << ", " << gripper_width_range[1] << "]  # [min, max] gripper width in meters\n";
  config_file << "min_inliers = " << min_inliers << "  # minimum number of inliers for clustering\n";
  config_file << "num_selected = " << num_selected << "  # number of selected grasps\n\n";
  
  // Close the file and return the path
  config_file.close();
  
  RCLCPP_INFO(node->get_logger(), "Created GPD config file at %s", config_file_path.c_str());
  return config_file_path;
}

void ROSParameters::getPlanningParams(
  rclcpp::Node * node,
  GraspPlanner::GraspPlanningParameters & param)
{
  node->get_parameter_or("grasp_service_timeout", param.grasp_service_timeout_, 0);
  node->get_parameter_or("grasp_score_threshold", param.grasp_score_threshold_, 200);
  node->get_parameter_or("grasp_frame_id", param.grasp_frame_id_, std::string("base"));
  std::vector<double> approach;
  node->get_parameter_or("grasp_approach", approach,
    std::vector<double>(std::initializer_list<double>({0.0, 0.0, -1.0})));
  param.grasp_approach_ = tf2::Vector3(approach[0], approach[1], approach[2]);
  node->get_parameter_or("grasp_approach_angle", param.grasp_approach_angle_, M_PI);
  node->get_parameter_or("grasp_offset", param.grasp_offset_,
    std::vector<double>(std::initializer_list<double>({0.0, 0.0, 0.0})));
  node->get_parameter_or("grasp_boundry", param.grasp_boundry_,
    std::vector<double>(std::initializer_list<double>({-1.0, 1.0, -1.0, 1.0, -1.0, 1.0})));
  node->get_parameter_or("eef_offset", param.eef_offset, 0.154);
  node->get_parameter_or("eef_yaw_offset", param.eef_yaw_offset, 0.0);
  node->get_parameter_or("grasp_min_distance", param.grasp_min_distance_, 0.06);
  node->get_parameter_or("grasp_desired_distance", param.grasp_desired_distance_, 0.1);

  // gripper parameters
  std::vector<double> finger_opens, finger_closes;
  node->get_parameter_or("finger_joint_names", param.finger_joint_names_,
    std::vector<std::string>(std::initializer_list<std::string>({std::string("panda_finger_joint1"),
      std::string("panda_finger_joint2")})));
  node->get_parameter_or("finger_positions_open", param.finger_points_open_.positions,
    std::vector<double>(std::initializer_list<double>({-0.01, 0.01})));
  node->get_parameter_or("finger_positions_close", param.finger_points_close_.positions,
    std::vector<double>(std::initializer_list<double>({-0.0, 0.0})));
}

}  // namespace grasp_ros2
