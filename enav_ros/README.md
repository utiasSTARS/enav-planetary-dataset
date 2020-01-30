# enav_ros package

Lightweight ROS package to interact and visualize data from the energy-aware planetary navigation dataset. First make sure you read the project's main [README.md](https://github.com/utiasSTARS/enav-planetary-dataset/blob/master/README.md) located in the root directory of this repository for preliminary instructions.

## Visualization of rover's reference pose estimates

To visualize the pose estimates generated through the combination of VINS-Fusion (stereo imagery + GPS data), IMU data and reference elevation data contained in a rosbag (`run1_base.bag` for example), start the main launch file the following way:


```sh
roslaunch enav_ros main.launch bag:=/complete/path/to/run1_base.bag map_dir:/path/to/map/directory start_time:=55.0
# Then, when everything loaded, hit spacebar to play the rosbag
```

Arguments:

* `bag`: the complete path to the base .bag file desired
* `map_dir` (optional, default is `false`): the complete path to the directory containing the map (.tif) files. If unspecified, no map will be loaded.
* `start_time` (optional, default is 0): the start time (seconds) into the rosbag. This is particularly useful when visualizing the reference pose estimates, since it allows the user to skip the initialization phase of VINS-Fusion. For convenience, here are the `start_time` values to skip the VINS-Fusion initialization for each run:

Run 1 | Run 2 | Run 3 | Run 4 | Run 5 | Run 6
--- | --- | --- | --- | --- | ---
55.0 | 2 | 3 | 4 | 20.0 | 38.0

The above command will load a pre-configured RViz windows with a model of the rover and corresponding TF tree. Note that at first, the default view will be fixed on the origin of the terrain ("csa_origin"). To play the bag, hit spacebar in the same terminal window. Once the bag runs (and position estimations are generated), the warnings/errors related to the "odom" frame should disappear.

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
roslaunch enav_ros main.launch bag:=/complete/path/to/run1_base.bag map_dir:/path/to/map/directory start_time:=55.0
# Then, when everything loaded, hit spacebar to play the rosbag
```