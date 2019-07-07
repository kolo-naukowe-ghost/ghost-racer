#include "VideoStreamer.hpp"
#include <ros/ros.h>

VideoStreamer::VideoStreamer()
{
    ROS_INFO("Video streamer\n");
}

bool  VideoStreamer::initialize()
{
    ROS_INFO("Initialized\n");
    return true;
}