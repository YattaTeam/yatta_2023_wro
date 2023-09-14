import cv2
import numpy as np
from time import sleep
from picamera import PiCamera
from picamera.array import PiRGBArray


class ColorsCoordinations:

    def __init__(self, brightness: int = 60, awb_mode="auto"):
        self.brightness = brightness
        self.awb_mode = awb_mode
        self.resolution = (1008, 400)
        self.framerate = 32
        self.crop_width = 220

        self.x, self.y, self.width, self.height = -1, -1, -1, -1
        self.coords = (-1, -1)

        self.rawCapture = None
        self.camera: PiCamera = PiCamera()
        self.__initCamera()
        self.frames = self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True)

    def __initCamera(self):
        self.camera.resolution = self.resolution
        self.camera.framerate = self.framerate
        self.camera.brightness = self.brightness
        self.camera.awb_mode = self.awb_mode
        self.rawCapture = PiRGBArray(self.camera, size=self.resolution)
        sleep(1)

    def get_coords(self, lower: np.ndarray, upper: np.ndarray):
        for frame in self.frames:
            imageFrame = frame.array

            key = cv2.waitKey(1) & 0xFF
            self.rawCapture.truncate(0)
            if key == ord("q"):
                break

            frame_shape = imageFrame.shape
            new_width = frame_shape[1] - self.crop_width * 2

            right_crop = frame_shape[1] - self.crop_width

            imageFrame = imageFrame[:, self.crop_width: right_crop]
            hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

            # lower = np.array([l1, l2, l3], np.uint8)
            # upper = np.array([u1, u2, u3], np.uint8)
            mask = cv2.inRange(hsvFrame, lower, upper)
            kernal = np.ones((5, 5), "uint8")

            mask = cv2.dilate(mask, kernal)
            red = cv2.bitwise_and(imageFrame, imageFrame,
                                  mask=mask)

            contours, hierarchy = cv2.findContours(mask,
                                                   cv2.RETR_TREE,
                                                   cv2.CHAIN_APPROX_SIMPLE)

            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if (area > 800):
                    self.x, self.y, self.width, self.height = cv2.boundingRect(
                        contour)

            self.coords = (self.x, self.y)
            self.x = self.y = -1

            return self.coords

    def get_area(self, lower: np.ndarray, upper: np.ndarray):
        for frame in self.frames:
            area = 0
            imageFrame = frame.array

            key = cv2.waitKey(1) & 0xFF
            self.rawCapture.truncate(0)
            if key == ord("q"):
                break

            hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsvFrame, lower, upper)
            kernal = np.ones((5, 5), "uint8")

            mask = cv2.dilate(mask, kernal)
            red = cv2.bitwise_and(imageFrame, imageFrame,
                                  mask=mask)

            contours, hierarchy = cv2.findContours(mask,
                                                   cv2.RETR_TREE,
                                                   cv2.CHAIN_APPROX_SIMPLE)

            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if (area > 750):
                    return area

            return 0

    def deInit(self):
        self.camera.close()


if __name__ == "__main__":
    colordetect = ColorsCoordinations()

    orangeLower = np.array([6, 19, 58], np.uint8)
    orangeUpper = np.array([25, 255, 255], np.uint8)

    reading = colordetect.get_coords(orangeLower, orangeUpper)
    print(reading)
