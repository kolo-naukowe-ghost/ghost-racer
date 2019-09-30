import numpy as np

class State:

    def __init__(self, *args):
        self.left_camera, self.center_camera, self.right_camera = args

    def as_numpy_array(self):
        return self.center_camera
