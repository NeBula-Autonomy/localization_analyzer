# Localization Analyzer

A repository for scripts and tools to analyze the performance of lidar odometry and the SLAM system.
Primarily used for [LOCUS](https://github.com/NeBula-Autonomy/LOCUS) and [LAMP](https://github.com/NeBula-Autonomy/LAMP) evaluation.
The package is a wrapper for awesome trajectory analyzing tool [evo](https://github.com/MichaelGrupp/evo).


# How to store data in order to run the scripts

To run the scripts smoothly you should have the following folder structure for dataset that you are going to evaluate. Assuming the folder is called ```test_localization_analyzer_dataset```. In this folder you should have the following folders and files

```
+-- ground_truth
|   +-- odometry.bag
+-- specific_method_run_number
|   +-- cpu.bag
|   +-- delay.txt
|   +-- mem.bag
|   +-- odometry.bag
|   +-- rate.txt
+-- specific_method2_run_number
|   +-- cpu.bag
|   +-- delay.txt
|   +-- mem.bag
|   +-- odometry.bag
|   +-- rate.txt
+-- specific_method3_run_number
|   +-- cpu.bag
|   +-- delay.txt
|   +-- mem.bag
|   +-- odometry.bag
|   +-- rate.txt
```
Folder naming:
- ```specific_method```, ```specific_method2```, ```specific_method3``` corresponds to the names in ```specific_methods``` in [config/analyzer_settings.yaml](https://github.com/NeBula-Autonomy/localization_analyzer/blob/main/config/analyzer_settings.yaml)

- ```run_number``` - corresponds to the test name that you specified for your tests (see [run_analyzer.yaml](https://github.com/NeBula-Autonomy/localization_analyzer/blob/main/scripts/run_analyzer.yaml) for example)

- ```ground_truth``` - folder where you store bag folder with the ground truth (needs to be called ```odometry.bag```)

Files description:
- ```cpu.bag``` and ```mem.bag``` - produced from ```monitor.py```. See [LOCUS scripts]() how to use it.

- ```delay.txt``` - dekay of the odometry topic
```
rostopic delay <odometry_topic> -w3 >> /path/to/test_localization_analyzer_dataset/specific_method_run_number/delay.txt
```

- ```rate.txt``` - rate of the odometry topic
```
rostopic hz <odometry_topic> -w3 >> /path/to/test_localization_analyzer_dataset/specific_method_run_number/rate.txt
```

- ```odometry.bag``` - recorded odometry information
```
rosbag record -O /path/to/test_localization_analyzer_dataset/specific_method_run_number/odometry.bag <odometry_topic>
```

# How to run?

To run analyzer (see [run_analyzer.yaml](https://github.com/NeBula-Autonomy/localization_analyzer/blob/main/scripts/run_analyzer.yaml) for easy use)

```
python $(rospack find localization_analyzer)/scripts/analyzer.py <data/path/to/your/folder> <robot_name> <robot_odometry_topic> <test_name>
```

```<data/path/to/your/folder>``` - path to the main folder where your recorded bag data is (e.g. path to ```test_localization_analyzer_dataset```)

```<robot_name>``` - robot namespace

```<robot_odometry_topic>``` - topic of the odometry (needs to contain robot namespace)

```<test_name>``` - name of the test

# Content description

## config
```analyzer_settings.yaml``` - config file that specify what metrics should be evaluated for single robot odometry 

```base_analyzer_settings.yaml``` 

## scripts
```analyzer.py``` - the entrypoint of single robot analyzer

```base_analyzer.py``` - the entrypoint of multi robot analyzer

```pose_graph_loop_closure_analysis.py```

```run_analyzer.yaml``` - [tmux](https://tmuxp.git-pull.com/) script for the single robot analyzer



## utilities folder
```aggregate_odometries.py``` - script to aggregate user requested odometries along with ground truth odometry.

```pose_graph_to_odometry.py```

```segment_aggregated_odometries.py```

```utilities.py``` - module with all the extra functionalities used for analysis

```boxplot_from_npz.py``` - drawing boxplt from npz

```compute_distances_travelled.py``` - compute distance travelled

```get_dataset_duration.py``` - get dataset duration

```get_distance_traveled_from_aggregated_odometries.py```  

```loop_closure_eval.py```

```volume_over_time.py```

```plot_factor_covariance.py```

```plot_lc_ftness_hist.py```

```plot_lc_log.py```

```plot_loop_candidates.py```

```plot_odom_covariance.py```

```plot_volume_size_from_map_info.py```

# Do we need that?


```recover_fiducial_calibration_from_pose_graph.py```

# Removed

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
