#include "VideoStreamer.hpp"
#include "CameraStreamer.hpp"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <string>

int main(int argc, char **argv)
{
    const int targetFramerate = 25;
    const int targetWidth = 500;
    const int targetHeight = 500;
    Size target_resolution = Size(targetWidth, targetHeight);
    int publishedFramesCount = 0;

    ros::init(argc, argv, "video_streamer");

    ros::NodeHandle nodeHandle;
    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Publisher imagePublisher = imageTransport.advertise(VideoStreamer::TopicName, 1);

    VideoStreamer *cameraStreamReader = new CameraStreamer();

    if(!cameraStreamReader->initialize())
    {
        delete cameraStreamReader;
        return 0;
    }

    // trying to set targetFPS
    if(!cameraStreamReader->setFramerate(targetFramerate))
    {
        // this is not so serious
        ROS_WARN("Failed to set target framerate.");
    }
    if(!cameraStreamReader->setStreamSize(targetWidth, targetHeight))
    {
        ROS_ERROR("Failed to set target witdth or height.");
    }

    double real_fps = cameraStreamReader->getFPS();
    ROS_INFO_STREAM("Stream real FPS is " << real_fps);
    ros::Rate loop_rate(real_fps);

    Mat frame;
    while(ros::ok())
    {
        if(!cameraStreamReader->getFrame(frame))
        {
            ROS_WARN("Empty frame fetched, waiting.");
            loop_rate.sleep();
            continue;
        }

        bool isTargetSize = frame.size() == target_resolution;

        if(!isTargetSize)
        {
            resize(frame, frame, target_resolution, 0, 0, INTER_NEAREST);
        }

        auto header = std_msgs::Header();
        header.stamp = ros::Time::now();

        auto imgPtr = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        imagePublisher.publish(*imgPtr);

        publishedFramesCount++;
        // ROS_INFO_STREAM(std::to_string(publishedFramesCount));

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Closing video streamer.");
    delete cameraStreamReader;

    ros::shutdown();
    ros::waitForShutdown();

    return 0;
}