import cv2

class VideoStreamer(object):
    device = '/dev/video0'
    topic_name = 'video_streamer'

    def __init__(self, fps, width, height):
        self.fps = fps
        self.w = width
        self.h = height
        self.capture = cap = cv2.VideoCapture(VideoStreamer.device)
        self.initialized = False
        self.initialize_stream()

    def get_stream(self):
        pass

    def get_fps(self):
        '''
        This implementation is only valid for a stream from a file.
        CameraStreamer uses its own method.
        '''
        return self.capture.get(cv2.CAP_PROP_FPS)

    def initialize_stream(self):
        if not self.capture.isOpened():
            self.initialized = False
        else:
            self.initialized = True

        return self.initialized