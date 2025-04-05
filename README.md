# The CU-Multi Dataset
<p align="center">
  <img src="./assets/banner_light.png" alt="banner" height="400">
</p>

The CU-MULTI Dataset: A dataset aimed to support multi-robot map-merging, inter-robot place recognition and loop-closure. **If you would like to download the dataset, you can do so [here](https://drive.google.com/drive/folders/1lrhCDy2flNDyyPkKeTmA8tNgFj4JxSwi?usp=sharing).**

## Converting to ROS1/ROS2 bag formats
*Please Note*: The order of this is likely to change with a cleaner layout, but this will quickly get you started. 

To convert the dataset to ROS1 bag files, you can use the provided Docker containers in the **dataset_tools** directory following the steps below. Before proceeding further, please make sure you have Docker installed. You can install Docker following the instructions at [this link](https://docs.docker.com/engine/install/). If you are using Arch, you can follow the instructions [here](https://itsfoss.com/install-docker-arch-linux/).

Before installing either (or both) containers, make sure to keep a note on where you download the dataset and where you have cloned this repository (you will need the global location of both). If you are using a unix-based OS, you can find the global location of either by entering the directory and typing ***pwd***.

Update the ***SCRIPTS_DIR*** and ***DATASETS_DIR*** variables ***run_and_enter_container.sh*** bash script with the path of the ***conversion_scripts*** directory (there's one in the ros1 and ros2 folders) you will be using and where you have saved the CU-Multi dataset, respectively. 

### Convert to ROS1 rosbag format

Install and enter the Docker container
1. > cd dataset_tools
2. > cd ros1
3. > sudo bash docker/build_docker.sh
4. > sudo bash run_and_enter_container.bash
5. > source /opt/ros/noetic/setup.bash

While you are in the container, run ***ros_core***. If you want to do this in a tmux session, even better. Open a new window and repeat entering the container like above if not using a tmux session. Now enter ***/root/conversion_scripts*** directory. In this directory is the Python file for converting the unstructured CU-Multi dataset into a ROS1 rosbag file.

First, edit the Python file with the environment and corresponding robot you would like to convert.

Now run the Python file with:

> python3 make_ros1_bag.py. 

### Convert to ROS2 rosbag format

Install and enter the Docker container
1. > cd dataset_tools
2. > cd ros2
3. > sudo bash docker/build_docker.sh
4. > sudo bash run_and_enter_container.bash

Enter the ***/root/conversion_scripts*** directory. In this directory is the Python file for converting the unstructured CU-Multi dataset into a ROS2 db3 file.

First, edit the Python file with the environment and corresponding robot you would like to convert.

Now run the Python file with:

> python3 make_ros2_bag.py. 