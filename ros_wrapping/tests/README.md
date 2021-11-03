# lidar_slam_test

This is a test node to ensure the good behavior of the SLAM after modifications. It relies on reference data which must be created by a SLAM reference branch.
No reference data nor dataset is supplied with this package for now. It is mainly used for the Kitware SLAM CI.

## Basic usage

**1. Create the reference data**

```bash
roslaunch lidar_slam_test slam.launch test_data:="path/to/rosbag.bag" res_path:="path/to/folder/where/to/store/log/files"
```

This creates 2 files : _path/to/folder/where/to/store/log/files/Poses.csv_ and _path/to/folder/where/to/store/log/files/Evaluators.csv_. The first file contains the 6D poses for each frame received. The second one contains some confidence estimators relative to each pose: the overlap, the number of matches and the computation time.

**2. Run the comparison**

```bash
roslaunch lidar_slam_test slam.launch test_data:="path/to/rosbag.bag" ref_path:="path/to/reference/folder" res_path:="path/to/folder/where/to/store/log/files"
```

Where _"path/to/reference/folder"_ refers to the previous log storage folder or to the new folder where the reference log data have been store.
This compares each pose and relative confidence estimator values and computes some difference with reference log data. It qualifies the success or failure of the test relatively to some thresholds (see next section) and print some valuable metrics.

## Options

The config file [`params/eval.yaml`](params/eval.yaml) contains some threshold parameters to qualify the results :

* **_time_threshold_** : this value limits the mean computation time difference with reference for the default number of threads.
* **_angle_threshold_** : this value limits the angle difference (in degrees) on EACH pose.
* **_position_threshold_** : this value limits the position difference (in meters) on EACH pose.

**NOTE :** One can optionally activate the verbose option : ```verbose:=true``` in the roslaunch command to print the difference with reference for each pose in order to debug.

## CI

The basic CI usage is the following :
* **1st job**

   1. Checkout master reference branch
   2. Create the reference data

* **2nd job (dependent)**
   3. Checkout the test branch
   4. Run the comparison