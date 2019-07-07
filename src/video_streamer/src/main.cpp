#include "VideoStreamer.hpp"
#include <ros/ros.h>
#include <image_transport/image_transport.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "video_streamer");

    ros::NodeHandle nodeHandle;
    image_transport::ImageTransport imageTransport(nodeHandle);
    image_transport::Publisher imagePublisher = imageTransport.advertise(VideoStreamer::TopicName, 1);
    
    VideoStreamer cameraStreamReader;
    
    cameraStreamReader.initialize();
}