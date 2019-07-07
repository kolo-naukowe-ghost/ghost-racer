#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <string>
#include <iostream>

class VideoStreamer
{
    public:
        static constexpr const char* TopicName = "video_streamer";
        VideoStreamer();
        bool initialize();
};