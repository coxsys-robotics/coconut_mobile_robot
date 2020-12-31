include "test_slam.lua"

-- Local SLAM
TRAJECTORY_BUILDER.pure_localization = true
-- TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 3 -- 1
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 200 -- 160
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2 -- 10
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 8 -- 40

-- TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true --false
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1 -- 0.1
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.) -- 20.
-- TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
-- TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)

-- Global SLAM
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60

POSE_GRAPH.optimize_every_n_nodes = 1 --90
-- POSE_GRAPH.global_sampling_ratio = 0.003 -- 0.003
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.03 --0.3
-- POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight =  1e2
-- POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 2e2
-- POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e2
-- POSE_GRAPH.optimization_problem.odometry_rotation_weight = 2e2
-- POSE_GRAPH.constraint_builder.max_constraint_distance = 20 --default=15
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7 --default=7
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 15 --default=7
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(90.) --default=30.
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10e0 -- default=10
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1e0 -- default = 1
-- POSE_GRAPH.matcher_translation_weight = 5e2 -- default=5e2
-- POSE_GRAPH.matcher_rotation_weight = 1.6e3 -- default=1.6e3



return options
