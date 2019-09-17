This repository contains code of `GHOST-racer` a self-driving car built by the members of the `GHOST` scientific group from Poznań University of Technology.

# Setup
We use `ROS` - Robot Operating System, running on the `Ubuntu 18.04`.
## Installation process
1. Install `ROS melodic` following the instructions from the [official ROS webpage](http://wiki.ros.org/melodic/Installation/Ubuntu). For the development purposes it is recommended to install `ros-melodic-desktop-full` version.
2. Clone the repository:
```
git clone https://github.com/kolo-naukowe-ghost/ghost-racer.git
cd ghost-racer
```
3. Build the project
```
catkin_make
# Set various environment variables and sources additional
# environment hooks.
source devel/setup.sh
```
4. Ensure there is a webcam connected to your PC. Run `roscore` and the launch existing 2 packages:
```
roscore & # if wasn't launched before

# run video_streamer
rosrun video_streamer video_streamer_node

# open a new console window
# run video_reader
rosrun stub_video_reader stub_video_reader_node
```

Now, you should be able to see some performance checks in the `video_reader` console being typed every 100 frames. The connection between `video_streamer` node and `video_reader` node is present.

In order to create new package (node) follow the instruction from [the official ROS tutorial](http://wiki.ros.org/ROS/Tutorials/CreatingPackage).

# Running

The easiest way to run the ghost-racer is to pull a docker image from a docker-hub and run script that will automatically run docker with all required docker's parameters:

    docker pull jakubtomczak/ghost-racer
    sudo ./docker_run.bash