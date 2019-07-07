#pragma once
#include "VideoStreamer.hpp"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class CameraStreamer: public VideoStreamer
{
    public:
        CameraStreamer(){};
        ~CameraStreamer()
        {
            ROS_INFO("Destructing CameraStreamer");
        }
        bool initialize() override;
        bool getFrame(Mat &frame) override;
    private:
        VideoCapture cameraCapture;
};