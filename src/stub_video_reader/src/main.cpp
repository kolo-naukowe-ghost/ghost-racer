#include "VideoReader.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    VideoReader videoReader;

    ros::init(argc, argv, "video_reader");
    ros::NodeHandle nodeHandleVideoReader;
    image_transport::ImageTransport imageTransport(nodeHandleVideoReader);

    auto frameFetchedCallback = imageTransport.subscribe(VideoReader::TopicNameToRead, 1, &VideoReader::frameFetchedCallback, &videoReader);
    ROS_INFO_STREAM("Subscribed to '" << VideoReader::TopicNameToRead << "' topic, waiting for images.");

    // ros::Rate loop_rate(1000);
    while(ros::ok())
    {
        ros::spinOnce();
        // loop_rate.sleep();
    }

    ros::shutdown();
    ros::waitForShutdown();

    return 0;
}