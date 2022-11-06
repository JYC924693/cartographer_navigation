-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "base_footprint",
  odom_frame = "odom",
  provide_odom_frame = true,  -- cartographer自带的里程计
  publish_frame_projected_to_2d = false,
  publish_tracked_pose = true,  -- 发布小车位置
  
  use_pose_extrapolator = false,
  use_odometry = false,  -- 自己使用的里程计
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- MAP_BUILDER.use_trajectory_builder_2d = true
-- TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 40
TRAJECTORY_BUILDER_2D.min_range = 0
TRAJECTORY_BUILDER_2D.max_range = 12
TRAJECTORY_BUILDER_2D.min_z = 0

TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.015  -- modify
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.015
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 12

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(10.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 0.1

TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.015
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 50
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 12.  -- yuan 30
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 1.

POSE_GRAPH.optimize_every_n_nodes = 0
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1  --修改 0.2
POSE_GRAPH.constraint_builder.max_constraint_distance = 6  --修改 
POSE_GRAPH.constraint_builder.min_score = 0.65  -- 修改 原0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55
POSE_GRAPH.global_sampling_ratio = 0.001  --0.005

POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 50 -- 修改 原注释

return options
