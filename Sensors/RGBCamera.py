import cv2
import threading
import os, sys
import numpy as np
import time


class RGBCamera(object):

    def __init__(self, camera_id: int = 0, demo_mode: bool = False):
        """
        Function:
            Initialize the camera. The module will automatically detect the camera and keep reading images. Frequency
            will be over 60Hz. You can get the latest frame by using the get_rgb_image().
        Parameter:
            camera_id: assign the specific camera with the id numer (default 0)
            demo_mode: a bool parameter to open the demo mode, which is a small window to display the image in real time
        Return:
            None
        """
        self.cam = cv2.VideoCapture(camera_id)
        self.image = np.zeros((480, 640, 3))
        self.demo_mode = demo_mode

        self.cam_thread = threading.Thread(target=self.CapturingLoop, args=())
        self.cam_thread.start()

    def CapturingLoop(self) -> None:
        """
        Function:
            Continuously recording the camera and updating the image.
        Parameter:
            None
        Return:
            None
        """
        while cv2.waitKey(1) == -1:
            # Press any key to quit
            success, self.image = self.cam.read()
            if self.demo_mode:
                cv2.imshow("RGB", self.image)
        self.cam.release()
        cv2.destroyAllWindows()

    def get_rgb_image(self) -> np.ndarray:
        """
        Get one RGB image from the camera class.

        :return: one [480, 640, 3] numpy array
        """
        return self.image


if __name__ == "__main__":
    Camera_instance = RGBCamera(0, demo_mode=True)
    Camera_instance.CapturingLoop()
