#pragma once
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

class VideoStreamer
{
    public:
        static constexpr const char* TopicName = "video_streamer";
        VideoStreamer();
        virtual ~VideoStreamer()
        {
            ROS_INFO("Destructing VideoStreamer");
        }
        virtual bool initialize() = 0;
        virtual bool getFrame(Mat &frame) = 0;
};