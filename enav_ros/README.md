# enav_ros package

Lightweight ROS package to interact and visualize data in the energy-aware planetary navigation dataset.

First, read the project's main [README.md](https://github.com/utiasSTARS/enav-planetary-dataset/blob/master/README.md) and set up / run a Docker container. The following commands need to run inside the container.

## Visualization of rover's reference pose estimates

To visualize the pose estimates generated through the combination of VINS-Fusion (stereo imagery + GPS data), IMU data and reference elevation data contained in a rosbag (`run1_base_new.bag` for example), start the main launch file the following way:

```sh
roslaunch enav_ros main.launch bag:=/enav_dataset/run1_base_new.bag map_dir:=/enav_dataset/maps start_time:=55.0
# Then, when everything loaded, hit spacebar to play the rosbag
```

The above command will load a pre-configured RViz windows with a model of the rover and corresponding TF tree. Note that at first, the default view will be fixed on the origin of the terrain ("csa_origin"). To play the bag, hit spacebar in the same terminal window.

Arguments:

* `bag`: the complete path to the base .bag file desired (recall: if you followed the provided Docker instructions, the dataset directory should be mounted in the container at `/enav_dataset`)
* `map_dir` (optional, default is `false`): the complete path to the directory containing the map (.tif) files. If unspecified, no map will be loaded.
* `start_time` (optional, default is 0): the start time (seconds) into the rosbag. This is particularly useful when visualizing the reference pose estimates, since it allows the user to skip the initialization phase of VINS-Fusion. For convenience, here are the `start_time` values to skip the VINS-Fusion initialization for each run:

Run 1 | Run 2 | Run 3 | Run 4 | Run 5 | Run 6
--- | --- | --- | --- | --- | ---
55.0 | 20.0 | 25.0 | 25.0 | 20.0 | 38.0

## Merging all the data (including point clouds) into one rosbag

As mentioned on the dataset's web page, the point clouds of each run were saved in a separate rosbag. While it is possible to manually start both bags (`base` and `clouds_only`) more or less simultaneously in separate terminal windows, another option is to merge them into a combined rosbag file using the [bagedit package](https://github.com/MHarbi/bagedit) (already installed in the image):

```sh
# Terminal 1 inside container
roscore

# Terminal 2 inside container (open with: `docker exec -it enav bash` on host computer)
rosrun bagedit bagmerge.py -o /enav_dataset/run1_combined.bag /enav_dataset/run1_base_new.bag /enav_dataset/run1_clouds_only.bag
```

Notes: leave enough free space on your device for the merged rosbag (equivalent to the combined size of the `base` and `clouds_only` bags).

The merged rosbag data can then be visualized following a similar procedure to that described above:

```sh
roslaunch enav_ros main.launch bag:=/enav_dataset/run1_combined.bag map_dir:/enav_dataset/maps start_time:=55.0
# Then, when everything loaded, hit spacebar to play the rosbag
```
