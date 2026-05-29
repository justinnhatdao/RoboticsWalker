-- Cartographer SLAM configuration for the TurtleBot3 waffle with a 2D LiDAR.
-- These are the settings I tuned to get clean maps in my guide_house world.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",  -- Frame Cartographer tracks pose in, must match the URDF
  published_frame = "odom",     -- Cartographer publishes the map->odom transform
  odom_frame = "odom",
  provide_odom_frame = false,   -- Gazebo already provides odometry, so I don't need Cartographer to
  publish_frame_projected_to_2d = true,  -- Flatten to 2D since this is a flat-floor environment
  use_odometry = true,          -- Fuse wheel odometry with the scan for better pose estimates
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,          -- One 2D LiDAR on the waffle
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

-- Use the 2D trajectory builder since this is a flat-floor mapping task
MAP_BUILDER.use_trajectory_builder_2d = true

-- min_range matches the waffle LiDAR's minimum valid reading so bad close-range data is ignored
TRAJECTORY_BUILDER_2D.min_range = 0.12
-- max_range capped at 3.5 m because my world's rooms are small and longer rays added noise
TRAJECTORY_BUILDER_2D.max_range = 3.5
-- Rays beyond max_range are treated as free space out to this length
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
-- The waffle sim doesn't publish IMU data, so I disabled this to avoid Cartographer waiting for it
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- Online correlative scan matching improves loop closure quality while mapping in real time
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- Only update the pose when the robot rotates more than 0.1 degrees to reduce noise from vibration
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- Disable periodic global optimization during mapping so the map doesn't shift mid-run,
-- I run the final optimization after the full map is captured
POSE_GRAPH.optimize_every_n_nodes = 0
-- Minimum score a scan match must hit before it becomes a loop closure constraint
POSE_GRAPH.constraint_builder.min_score = 0.65
-- Higher threshold for global localization to avoid false loop closures in similar-looking hallways
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

return options
