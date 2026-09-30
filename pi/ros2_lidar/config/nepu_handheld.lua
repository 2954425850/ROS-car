-- Handheld 2D SLAM for the nepu robot's RPLIDAR A1M8.
--
-- No IMU, no wheel odometry: the operator carries the robot and cartographer
-- tracks motion with its own online correlative scan matcher (the standard
-- revo_lds-style handheld setup). Walk slowly; sharp wrist turns outrun the
-- 7.6 Hz scan and break the match.
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "laser",
  published_frame = "laser",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_point_clouds = 0,
  -- Required by the node code but no longer in the stock defaults; values
  -- mirror backpack_2d.lua.
  num_subdivisions_per_laser_scan = 10,
  use_pose_extrapolator = true,
  landmarks_sampling_ratio = 1.,
  lookup_transform_timeout_sec = 0.3,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 12.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.

-- The whole point of handheld mode: exhaustive scan matching against the
-- running submap, with a search window wide enough to bridge gaps between
-- scans when the carrier changes pace, but not so wide it locks onto walls.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(35.)

POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.optimization_problem.huber_scale = 1e2

return options
