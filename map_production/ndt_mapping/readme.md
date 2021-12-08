## Quick Start
This is Baidu apollo offline mapping tool. 

## Environment
The environment for creating the map is as follows, you need to be equipped with lidar and GNSS(IMU+GPS). 
* RoboSense RS-LiDAR-32
* GNSS
* Apollo 5.0

## Collect data
First you need to collect the sensor data needed for mapping. If your vehicle has been installed with Apollo5.0, you can use below command to record the bag.
```
cyber_recorder record -c imu_topic localization_pose_topic lidar_topic
```
After collecting the data, you can start making a map by following the steps below.

## How to run
Compile the code according to the following steps.
1. Copy the entire directory `ndt_mapping` to apollo's `modules/localization/msf/local_tool/`
2. Build the localization module in apollo

#### 1.Unzip the bag
Extract the pcd file and pose file from the bag. You can use multiple "--bag_file" to extract multiple bag files. The decompressed file is saved in `--out_folder`.
```
./bazel-bin/modules/localization/msf/local_tool/data_extraction/cyber_record_parser --bag_file=data/bag/20210514100819.record.00000 --bag_file=data/bag/20210514100819.record.00001 --out_folder=data --cloud_topic=/apollo/sensor/rs32/PointCloud2
```

#### 2.Poses interpolation
Interpolate the pose according to the external parameters and timestamp of the lidar. The corrected pose is saved in `--output_poses_path`.
```
./bazel-bin/modules/localization/msf/local_tool/map_creation/poses_interpolator --input_poses_path=data/pcd/odometry_loc.txt --ref_timestamps_path=data/pcd/pcd_timestamp.txt --extrinsic_path=modules/localization/msf/params/velodyne_params/velodyne64_novatel_extrinsics_example.yaml --output_poses_path=data/pcd/poses.txt
```

#### 3.NDT mapping
Use the following command to create a map, the result of the map is default saved in "data/output.pcd"
```
./bazel-bin/modules/localization/msf/local_tool/ndt_mapping/ndt_mapping
```

You can also set some parameters in the command line according to your needs 
```
./bazel-bin/modules/localization/msf/local_tool/ndt_mapping/ndt_mapping -output_file=xxx
```

The parameters list
```
// filter
-min_scan_range = 25.0  // the square of the min scan range
-max_scan_range = 10000.0  // the square of the max scan range
-min_add_scan_shift = 1.0  // the square of the min add scan length
-voxel_leaf_size = 2.0  // voxel leaf size

// ndt
-trans_eps = 0.01  // transformation epsilon
-step_size = 0.1  // step size
-ndt_res = 1.0    // ndt resolution
-max_iter = 30    // maximum iterations times

// map
-output_file = "data/output.pcd"  // map save file path
-workspace_dir = "data/pcd"       // work dir
```

## Example
The following is the result of the mapping of the underground parking lot.
![parking_lot](img/parking_lot.jpg)  


