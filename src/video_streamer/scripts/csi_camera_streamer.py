from video_streamer import VideoStreamer
import cv2
import rospy

class CsiCameraStreamer(VideoStreamer):
    def __init__(self, fps, width, height):
        super(CsiCameraStreamer, self).__init__(fps, width, height)


    def gstreamer_pipeline(self, fps, width, height, flip_method = 0):
        return ('nvarguscamerasrc ! '
        'video/x-raw(memory:NVMM), '
        'width=(int)%d, height=(int)%d, '
        'format=(string)NV12, framerate=(fraction)%d/1 ! '
        'nvvidconv flip-method=%d ! '
        'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
        'videoconvert ! '
        'video/x-raw, format=(string)BGR ! appsink'  % (width, height, fps, flip_method, width, height))

    def initialize_stream(self):
        gst = self.gstreamer_pipeline(self.fps, self.w, self.h)
        self.capture = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
        if not self.capture.isOpened():
            self.initialized = False
        else:
            self.initialized = True

        return self.initialized

    def get_stream(self):
        return self.capture.read()
