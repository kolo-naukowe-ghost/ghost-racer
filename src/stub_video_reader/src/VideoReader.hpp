#pragma once
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

class VideoReader
{
    public:
        VideoReader() {}
        ~VideoReader() {}
        void frameFetchedCallback(const sensor_msgs::ImageConstPtr &frame);
        static constexpr const char* TopicNameToRead = "video_streamer";

        Mat fetchedFrame;
        inline ros::Time getLastFrameTimestamp()
        {
            return newFrameTimestamp;
        }
        inline bool isNewFrameFetched()
        {
            return newFrameFetched;
        }
        inline unsigned long numberOfFramesFetched()
        {
            return framesCount;
        }


    private:
        ros::Time newFrameTimestamp;
        bool newFrameFetched = false;
        unsigned int framesCount = 0;
};