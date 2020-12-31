include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint", 
  published_frame = "base_footprint",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0, -- 1
  lookup_transform_timeout_sec = 0.1,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3, --5e-3 = 200 hz
  trajectory_publish_period_sec = 30e-3, --30e-3 = 33hz
  rangefinder_sampling_ratio = 1., --1v
  odometry_sampling_ratio = 1.0, --1
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1., --1
  landmarks_sampling_ratio = 1.,
}
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 6

-- trajectory_builder
TRAJECTORY_BUILDER_2D.min_range = 0.4
TRAJECTORY_BUILDER_2D.max_range = 30.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5 -- default = 5
TRAJECTORY_BUILDER_2D.min_z = 0.08
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- Local SLAM
TRAJECTORY_BUILDER.pure_localization = false
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 -- 1
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60-- 160
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1 --1 -- 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 42 --270 -- 40

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false --false
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1 -- 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.) -- 20.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)

-- These 3 เกี่ยวกับการใส่ตัวจุด ๆ สีดำในแมพ
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true -- default = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55 -- default = 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49 -- default = 0.49

-- Global SLAM
-- 2 ตัวนี้ส่งผลกับการ recovery ตอนเปิดหุ่น --เส้นสีเหลืองต้องชัดแค่ไหนถึงโผล่ในแมพ
POSE_GRAPH.constraint_builder.min_score = 0.7 --0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.75 --0.6 --ตัวนี้ทำให้วาร์ปง่ายขึ้น

POSE_GRAPH.optimize_every_n_nodes = 40 --90
POSE_GRAPH.global_sampling_ratio = 0.003 -- 0.003
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03 --0.3

-- 4 ตัวนี้ เลือกเชื่อ local slam กะ dometry
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight =  1e4
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 2e4
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e4
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 2e2

-- ปรับไปไม่ค่อยเห็นผล 
POSE_GRAPH.constraint_builder.max_constraint_distance = 15 --default=15
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 10 --default=7
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7 --default=7
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.) --default=30.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10e0 -- default=10
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1e2 -- default = 1
POSE_GRAPH.matcher_translation_weight = 5e2 -- default=5e2
POSE_GRAPH.matcher_rotation_weight = 1.6e3 -- default=1.6e3
return options
