// Copyright (c) 2019 Intel Corporation. All Rights Reserved
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

#include "grasp_library/ros2/consts.hpp"

namespace grasp_ros2
{

const char Consts::kTopicPointCloud2[] = "/segmenter/segmented_pointcloud";
const char Consts::kTopicDetectedObjects[] = "/ros2_openvino_toolkit/segmented_obejcts";
const char Consts::kTopicDetectedGrasps[] = "/grasp_library/clustered_grasps";
const char Consts::kTopicVisualGrasps[] = "/grasp_library/grasps_rviz";
const char Consts::kTopicTabletop[] = "/grasp_library/tabletop_points";

}  // namespace grasp_ros2
