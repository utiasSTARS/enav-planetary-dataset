# enav_ros package

Lightweight ROS package to interact and visualize data from the energy-aware planetary navigation dataset. First make sure you read the project's main [README.md](https://github.com/utiasSTARS/enav-planetary-dataset/blob/master/README.md) located in the root directory of this repository for preliminary instructions.

## Basic visualization of rover planar odometry (wheel encoders only)

To visualize the planar wheel odometry estimates contained in a rosbag (`run1_base.bag` for example), start the main launch file by providing the path to the rosbag as argument:

```sh
roslaunch enav_ros main.launch bag:=/complete/path/to/run1_base.bag
# Then, hit spacebar to play the rosbag
```

This will load a pre-configured RViz windows with a model of the rover and corresponding TF tree. At first, the default fixed frame ("odom") will not be published and the related warnings/errors can be ignored. To play the bag, hit spacebar in the same terminal window. Once the bag runs (and position estimations are generated), the warnings/errors related to the "odom" frame should disappear.

Notes:

- In the current implementation, the velocity estimates of the rover's base (obtained solely from encoder velocities) are integrated over time to provide approximate 2D rover positions using a basic implementation of the EKF node included in the robot_localization package.

- The rover base manufacturer recommends using a 1.875 multiplier on the angular (yaw) velocity estimates to take into account the inherent slip experienced by the skid-steer base. This multiplier was not included in the rosbags we are providing, causing oversteering in the odometry estimates compared to the reality.

## Merging all the data (including point clouds) into one rosbag

As mentioned on the dataset's web page, the point clouds of each run were saved in a separate rosbag for convenience. A "master" rosbag file can be created by merging a run's `base` rosbag with its corresponding `clouds_only` rosbag (both are already synchronized) using the [bagedit package](https://github.com/MHarbi/bagedit). Here's a sample usage of the package's `bagmerge.py` script:

```sh
# Terminal 1
roscore

# Terminal 2
rosrun bagedit bagmerge.py -o run1_master.bag run1_base.bag run1_clouds_only.bag
```

Notes:

- Make sure to have enough free space on your device for the merged rosbag (equivalent to the combined size of the `base` and `clouds_only` bags).

- The bagedit package works with Python2.7. An alternative to installing a Python2.7 environment (and getting it to run with Python3.7) is to fix the `bagmerge.py` script by fixing (or commenting out) its `print` functions (i.e. change every `print "text"` to `print("text")`).

The merged rosbag data can then be visualized following a similar procedure to that described above:

```sh
roslaunch enav_ros main.launch bag:=/complete/path/to/run1_master.bag
# Then, hit spacebar to play the rosbag
```
