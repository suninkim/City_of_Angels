import numpy as np

from vision import CameraCalibration


class VisionSystem:
    def __init__(self, config):
        depth_estimation_model_name = config["depth_estimation"]
        segmentation_model_name = config["segmentation"]

        camera_calibration = CameraCalibration(config["calibration"])

    def get_object_position(self, image):
        object_center_ux = self.segmentation(image)
        object_center_depth = self.depth_estimation(image, object_center_ux)

        object_center_xyz = self.convert_2d_to_3d_pose(
            object_center_ux, object_center_depth
        )

        return object_center_xyz

    def segmentation(self, image):
        return np.array([640, 480])

    def depth_estimation(self, image, ux):
        return 0.3

    def convert_2d_to_3d_pose(self, ux, depth):
        return np.array([0.2, 0.1, 0, 2])
