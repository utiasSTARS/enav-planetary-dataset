# enav-planetary-dataset

Energy-aware navigation dataset for planetary rovers, collected at the Canadian Space Agency's Mars Emulation Terrain. This dataset amounts to more than 1,200 metres of driving and includes:

* Driving power consumption, solar irradiance on the rover's top plane and clear-sky direct irradiance estimates;
* Panoramic stereo color imagery and high-resolution forward-facing single-channel imagery;
* Tri-axial inertial measurements (acceleration & angular velocities);
* Individual wheel encoder data and corresponding 2D odometry estimates;
* Ground-truth GPS-based position and enhanced pose estimation from IMU, GPS and visual data fusion;
* Sun vector orientation relative to the rover's pose estimates;
* Geo-referenced maps of the environment (RGB mosaic, elevation/slope/aspect models).  

![rover_main](media/rover_main.jpg)

**Authors**: Olivier Lamarre, Oliver Limoyo, Filip Marić, Dr. Jonathan Kelly  
**Affiliation**: [Space and Terrestrial Autonomous Robotic Systems (STARS) Laboratory](www.starslab.ca), University of Toronto  
**Maintainer**: Olivier Lamarre ([email](mailto:olivier.lamarre@robotics.utias.utoronto.ca))

This dataset is described in details in our paper [*The Canadian Planetary Emulation Terrain Energy-Aware Rover Navigation Dataset*](https://journals.sagepub.com/doi/10.1177/0278364920908922), which was accepted for publication in The International Journal of Robotics Research. Please [cite our work accordingly](#citation).

## Overview

The dataset is further described in the project's official [web page](http://www.starslab.ca/enav-planetary-dataset/) and can be downloaded from a dedicated [IEEE DataPort page](https://ieee-dataport.org/open-access/canadian-planetary-emulation-terrain-energy-aware-rover-navigation-dataset) (requires creating a free account). The dataset is available in both rosbag and human-readable formats. This repository provides tools to interact with the rosbag files; we let users interested in the human-readable-formatted data develop their own utilities.

We provide a Docker container in which rosbags can be played and visualized using our custom ROS package, or parsed using our python utility scripts:

![enav_ros](https://media.giphy.com/media/YlBFpSSD9qxbCCCpyp/giphy.gif)

Lastly, this dataset also includes four different aerial maps of the test environment at a resolution of 0.2 meters per pixel: color, elevation, slope magnitude and slope orientation maps. Every map is georeferenced and is available in a `.tif` format. Tools to broadcast them as ROS topics (as [grid_map](https://github.com/ANYbotics/grid_map) messages) or simply load them using Python are also included:

![enav_maps](https://media.giphy.com/media/j4w8J6OvbReBvQyqn0/giphy.gif)

## Setup

As the dataset was originally collected with ROS Kinetic with a Ubuntu 16.04 machine, we provide a Docker container to play rosbags and/or extract specific data streams with Python.

1. Store downloaded rosbags in a single directory and the georeferenced maps in a dedicated subdirectory.

```sh
tree /path/to/dataset/dir

├── run1_base_new.bag
├── run2_base_new.bag
├── run3_base_new.bag
├── run4_base_new.bag
├── ...
├── maps
    ├── aspect_utm_20cm.tif
    ├── dem_utm_20cm.tif
    ├── mosaic_utm_20cm.tif
    └── slope_utm_20cm.tif
```

> Note: obviously, you don't need to download all the rosbags - just the ones you need.

2. Clone the current repository and store the location of the downloaded dataset in a `.env` file in the project's root directory:

```sh
git clone https://github.com/utiasSTARS/enav-planetary-dataset.git
cd enav-planetary-dataset

echo "ENAV_DATASET_DIR='/path/to/enav_dataset/dir'" > .env
```

3. If not already done, [install Docker](https://docs.docker.com/engine/install/). Then, set up X server permissions:

```sh
xhost +local:root
```

Run a container and enter it:

```sh
docker compose run --rm enav
```

Note that the dataset directory is now mounted at `/enav_dataset` in the container.

Rosbag interactions are done from inside the container:

- Playing rosbags and launching ROS visualization interfaces is documented in the `enav_ros` package's [README file](enav_ros/README.md).

- Extracting data from rosbags with Python is documented in the `enav_utilities` package's [README file](enav_utilities/README.md).

## Citation

```txt
@article{lamarre2020canadian,
   author = {Lamarre, Olivier and Limoyo, Oliver and Mari{\'c}, Filip and Kelly, Jonathan},
   title = {{The Canadian Planetary Emulation Terrain Energy-Aware Rover Navigation Dataset}},
   journal = {The International Journal of Robotics Research},
   year = {2020},
   doi = {10.1177/0278364920908922},
   URL = {https://doi.org/10.1177/0278364920908922},
   publisher={SAGE Publications Sage UK: London, England}
}
```
