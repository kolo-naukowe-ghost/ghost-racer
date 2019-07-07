#include "VideoReader.hpp"
#include <string>

void VideoReader::frameFetchedCallback(const sensor_msgs::ImageConstPtr &frame)
{
    cv_bridge::CvImageConstPtr cv_pointer;
    // toCvShare returns an immutable pointer
    // use toCvCopy to be able to modify the frame
    cv_pointer = cv_bridge::toCvShare(frame, sensor_msgs::image_encodings::BGR8);
    fetchedFrame = cv_pointer->image;

    newFrameTimestamp = frame->header.stamp;

#if CHECK_PERFORMANCE
    if(framesCount == 0)
    {
        this->firstFrameTimestamp = newFrameTimestamp;
    }
#endif

    framesCount++;
    newFrameFetched = true;

#if CHECK_PERFORMANCE
    differencesSum += (ros::Time::now() - newFrameTimestamp).toSec();
    if(framesCount > 0 && framesCount % this->performanceCheckWindowLength == 0)
    {
        ROS_INFO_STREAM("Mean time for sending image " << differencesSum / performanceCheckWindowLength);

        this->lastFrameTimestamp = newFrameTimestamp;
        this->calculatePerformance();

        this->firstFrameTimestamp = newFrameTimestamp;
    }
#endif
}

#if CHECK_PERFORMANCE
void VideoReader::calculatePerformance()
{
    auto fps = performanceCheckWindowLength / (lastFrameTimestamp - firstFrameTimestamp).toSec();
    ROS_INFO_STREAM("Frame #" << this->framesCount << ", FPS:" << fps);
}
#endif