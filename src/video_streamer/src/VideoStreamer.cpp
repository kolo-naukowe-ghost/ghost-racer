#include "VideoStreamer.hpp"
#include <ros/ros.h>

VideoStreamer::VideoStreamer()
{
    ;
}

bool VideoStreamer::setFramerate(int fps)
{
    return capture.set(CV_CAP_PROP_FPS, fps);
}

bool VideoStreamer::setStreamSize(int w, int h)
{
    bool result = true;
    result &= capture.set(CV_CAP_PROP_FRAME_WIDTH, w);
    result &= capture.set(CV_CAP_PROP_FRAME_HEIGHT, h);
    return result;
}

double VideoStreamer::getFPS()
{
    capture.get(CAP_PROP_FPS);
}