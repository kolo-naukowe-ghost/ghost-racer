from video_streamer import VideoStreamer
import rospy

class CameraStreamer(VideoStreamer):
    def __init__(self, fps, width, height):
        super(CameraStreamer, self).__init__(fps, width, height)

    def get_stream(self):
        '''
        Returns ret, frame.
        '''
        return self.capture.read()

    def get_fps(self):
        samples = 100

        from time import time
        start = time()
        [self.capture.read() for i in range(samples)]
        end = time()
        return samples / (end - start)