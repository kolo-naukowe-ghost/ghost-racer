#include "CameraStreamer.hpp"

bool CameraStreamer::initialize()
{
    if(!cameraCapture.open(0))
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
    cameraCapture >> frame;
  
    if (frame.empty())
        return false;
    return true;
}