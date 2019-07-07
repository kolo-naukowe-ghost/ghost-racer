#include "CameraStreamer.hpp"

bool CameraStreamer::initialize()
{
    if(!capture.open(deviceID))
    {
        ROS_ERROR("Couldn't find camera.");
        return false;
    }
    else
    {
        ROS_INFO("Camera streamer initialized");
        return true;
    }
}

bool CameraStreamer::getFrame(Mat &frame)
{
    capture >> frame;

    if (frame.empty())
        return false;
    return true;
}