This repository contains code of `GHOST-racer` a self-driving car built by the members of the `GHOST` students organization from Poznań University of Technology.

# Launching `GHOST-racer`
We use `ROS` - Robot Operating System, running on the `Ubuntu 18.04`. In order to launch `GHOST-racer` one may either install `ROS` and run everything on the host machine or run a dedicated docker image. The latter method is recommended.

Generally, for starting the project we use small script called `run.sh` when executed it assumes we are in the root directory of the project. First of all it starts gazeboo using command `roslaunch autonomous_driving_world spawn_conde_in_competition_track_D1_D2_B1_P1.launch`. Secondly it tries to launch agent as soon as the `gzserver` is running. It waits up to 10 seconds when it fails to find `gzserver` it just quits.

One should remove `build` and `devel` directories prior to launching project with docker for the first time or when switching from running the project in docker to running the project on the host machine (files generated in image's environment may be quiet different from those ones created on the host machine).

## Using `docker` image
1. Clone the repository:
```
git clone https://github.com/kolo-naukowe-ghost/ghost-racer.git
cd ghost-racer
```
2. Pull dedicated docker image and run `GHOST-racer`
```
docker pull jakubtomczak/ghost-racer
sudo ./docker_run.bash
```
This command will run docker image with all necessary parameters, `GHOST-racer` will be launched using `run.sh` script that is located in the project's main directory. The container will be disposed after quitting `GHOST-racer`. It is possible to run agent or any other node on the host machine since docker is using host's network. This helps when debugging.
## By installing `ROS` on the host system
1. Install `ROS melodic` following the instructions from the [official ROS webpage](http://wiki.ros.org/melodic/Installation/Ubuntu). For the development purposes it is recommended to install `ros-melodic-desktop-full` version.
2. Install additional dependencies:
```
sudo apt-get install ros-melodic-ackermann-msgs
pip install rospkg
pip install defusedxml
```
3. Clone the repository:
```
git clone https://github.com/kolo-naukowe-ghost/ghost-racer.git
cd ghost-racer
```
4. Run `GHOST-racer` by typing:
```
./run.sh
```

### Host's programs installation issues
OpenCV installation (should be shipped with ROS) - https://www.pyimagesearch.com/2018/05/28/ubuntu-18-04-how-to-install-opencv/