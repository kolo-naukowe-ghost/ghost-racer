#include "VideoReader.hpp"
#include <string>

void VideoReader::frameFetchedCallback(const sensor_msgs::ImageConstPtr &frame)
{
    cv_bridge::CvImageConstPtr cv_pointer;
    // toCvShare returns an immutable pointer
    // use toCvCopy to be able to modify the frame
    cv_pointer = cv_bridge::toCvShare(frame, sensor_msgs::image_encodings::BGR8);
    fetchedFrame = cv_pointer->image;

    framesCount++;
    newFrameTimestamp = frame->header.stamp;
    newFrameFetched = true;

    // ROS_INFO_STREAM(std::to_string(framesCount));
}