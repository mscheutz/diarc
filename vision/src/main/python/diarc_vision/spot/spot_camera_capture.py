"""
This module provides functionality for capturing and processing images from Boston Dynamics Spot robot's cameras.
It includes utilities for converting image formats, displaying images, and accessing both RGB and depth camera data.
"""

import argparse
import logging
import numpy as np
import cv2
from bosdyn.api import image_pb2

from spot_wrapper.wrapper import SpotWrapper

logger = logging.getLogger(__name__)


def image_to_opencv(image, auto_rotate=True):
    """Convert an image proto message to an openCV image.

    Args:
        image: Proto message containing image data from Spot robot.
        auto_rotate (bool): Flag to enable automatic image rotation (currently disabled).

    Returns:
        tuple: Contains (numpy.ndarray: OpenCV-compatible image array, str: file extension)
    """
    num_channels = 1  # Assume a default of 1 byte encodings.
    if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
        dtype = np.uint16
        extension = '.png'
    else:
        dtype = np.uint8
        if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGB_U8:
            num_channels = 3
        elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGBA_U8:
            num_channels = 4
        elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8:
            num_channels = 1
        elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U16:
            num_channels = 1
            dtype = np.uint16
        extension = '.jpg'

    img = np.frombuffer(image.shot.image.data, dtype=dtype)
    if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
        try:
            # Attempt to reshape array into an RGB rows X cols shape.
            img = img.reshape((image.shot.image.rows, image.shot.image.cols, num_channels))
        except ValueError:
            # Unable to reshape the image data, trying a regular decode.
            img = cv2.imdecode(img, -1)
    else:
        img = cv2.imdecode(img, -1)

    # if auto_rotate:
    #     img = ndimage.rotate(img, ROTATION_ANGLE[image.source.name])

    return img, extension


def display_image(img: cv2.Mat, name: str):
    """Display an image in a named window using OpenCV.

    Args:
        img (cv2.Mat): Image to be displayed.
        name (str): Window name for the displayed image.
    """
    img_copy = img.copy()
    cv2.imshow(name, img_copy)


def populate_cam_param_list(img_response):
    """Extract camera parameters from image response.

    Args:
        img_response: Image response containing camera parameters.

    Returns:
        list: Camera parameters [width, height, focal_length_x, focal_length_y, 
              principal_point_x, principal_point_y].
    """

    width = img_response.shot.image.cols
    height = img_response.shot.image.rows

    intrinsics = img_response.source.pinhole.intrinsics
    f_x = intrinsics.focal_length.x
    f_y = intrinsics.focal_length.y
    c_x = intrinsics.principal_point.x
    c_y = intrinsics.principal_point.y

    return [width, height, f_x, f_y, c_x, c_y]


class SpotCameraCapture:
    """Class for capturing and processing images from Spot robot's cameras.

    This class provides methods to capture RGB and depth images from Spot robot
    and retrieve camera parameters.
    """

    def __init__(self, username: str, password: str, hostname: str, port='', name=''):
        """Initialize SpotCameraCapture with connection parameters.

        Args:
            username (str): Spot robot username.
            password (str): Spot robot password.
            hostname (str): Spot robot hostname/IP.
            port (str, optional): Connection port. Defaults to ''.
            name (str, optional): Robot name. Defaults to ''.
        """
        self.spot = SpotWrapper(
            username=username,
            password=password,
            hostname=hostname,
            port=port,
            robot_name=name,
            logger=logger,
            start_estop=False,
            use_take_lease=False,
            get_lease_on_action=False,
            continually_try_stand=False,
            rgb_cameras=True,
        )

    def get_rgb_image(self):
        """Capture RGB image from Spot's hand camera.

        Returns:
            numpy.ndarray: RGB image in OpenCV format.
        """
        img_response = self.spot.spot_images.get_hand_rgb_image()
        cv2_img, _ = image_to_opencv(img_response)
        return cv2_img

    def get_depth_aligned_image(self):
        """Capture depth-aligned image from Spot's hand camera.

        Returns:
            numpy.ndarray: Depth image in meters as float32 array.
        """
        img_bundle = self.spot.spot_images.get_depth_registered_images()
        cv2_img, _ = image_to_opencv(img_bundle.hand)
        cv2_scaled_img = cv2_img.astype(np.float32) / img_bundle.hand.source.depth_scale # convert mm to m
        return cv2_scaled_img

    def get_rgb_camera_params(self):
        """
        Returns the parameters of the RGB camera.

        This method provides a list containing the parameters for an RGB camera.
        The parameters include width, height, focal length, and principal point.

        Returns:
            list: A list containing the RGB camera parameters. The parameters
            are ordered as [width, height, fx, fy, cx, cy].
        """

        img_response = self.spot.spot_images.get_hand_rgb_image()

        return populate_cam_param_list(img_response)

    def get_depth_camera_params(self):
        """
        Returns the parameters of the Depth camera.

        This method provides a list containing the parameters for a Depth camera.
        The parameters include width, height, focal length, and principal point.

        Returns:
            list: A list containing the RGB camera parameters. The parameters
            are ordered as [width, height, fx, fy, cx, cy].
        """

        img_bundle = self.spot.spot_images.get_depth_registered_images()
        img_response = img_bundle.hand

        return populate_cam_param_list(img_response)

# This main is largely used for testing behaviors
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Spot Camera Capture')
    parser.add_argument('--hostname', required=True, help='spot ip address')
    parser.add_argument('--username', required=True, help='spot username')
    parser.add_argument('--password', required=True, help='spot password')
    args = parser.parse_args()

    spot_capture = SpotCameraCapture(args.username, args.password, args.hostname)

    while True:
        color_img = spot_capture.get_rgb_image()
        color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
        display_image(color_img, "color image")

        depth_aligned_img = spot_capture.get_depth_aligned_image()
        display_image(depth_aligned_img, "depth aligned image")

        # key = cv2.waitKeywaitKey(1) & 0xFF
        key = cv2.pollKey() & 0xFF
        if key == ord('q') or key == ord('Q'):
            exit(0)