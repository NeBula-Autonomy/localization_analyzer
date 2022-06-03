# localizer_gt_analysis
A repository for scripts and tools to analyse performance of LO and LAMP. 

See wiki for details.

# Content description

## config
```analyzer_settings.yaml```
```base_analyzer_settings.yaml```

## scripts
```analyzer.py```
```base_analyzer.py```
```pose_graph_loop_closure_analysis.py```
```run_analyzer.yaml``` 



## utilities folder
```aggregate_odometries.py``` - script to aggregate user requested odometries along with ground truth odometry. Output file: ```aggregated_odometries.bag```
```compute_distances_travelled.py``` - 
```get_dataset_duration.py``` - get dataset duration
```get_distance_traveled_from_aggregated_odometries.py``` 
```plot_odom_covariance.py```
```plot_volume_size_from_map_info.py```
```pose_graph_to_odometry.py```
```segment_aggregated_odometries.py```
```utilities.py```
```volume_over_time.py```

# Not sure yet
```boxplot_from_npz.py```
```plot_factor_covariance.py```
```loop_closure_eval.py```
```plot_lc_ftness_hist.py```
```plot_lc_log.py```
```plot_loop_candidates.py```
```recover_fiducial_calibration_from_pose_graph.py```

# Notes

Things that I assumed we do not need:
- utilities/adapt_initial_transforms.py
- utilities/ape_from_aggregated_odometries_parser.bash
- utilities/ape_from_aggregated_odometries.bash
- utilities/change_base_data_to_different_robot.py
- utilities/convert_base_data_to_another_robot.py
- utilities/extract_first_message_from_bag.py
- utilities/extract_last_msg_from_bag.py
- utilities/h1_start_b2.sh
- utilities/key_handling.py
- utilities/loam_cpu_mem_aggregator.py
- utilities/plot_ape_results.bash
- utilities/plot_rpe_results.bash
- utilities/renamer.py
- utilities/rpe_from_aggregated_odometries.bash
- utilities/shift_init_pose_base_data.py
- utilities/tf_to_odometry.py
- utilities/visualize_trajectories_from_aggregated_odometries.bash

- tmuxp_configs/gt_generation/*
- tmuxp_configs/old/*
- tmuxp_configs/* 

- scripts/benchmark_checklist.txt
- scripts/collect_multi_robot_data.sh
- scripts/run_benchmark.sh
- scripts/run_localization_analysis_with_packets.yaml
- scripts/run_localization_analysis.yaml

- config/dataset_settings.yaml
- config/base_replay.rviz
- config/base_replay_filtered.rviz