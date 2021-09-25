import pathlib
from unittest import TestCase

import cv2
import matplotlib.pyplot as plt
import numpy as np

from gem_lanenet.lanenet_w_line_fit import LaneNetWLineFit


class TestLaneNetWLineFit(TestCase):
    CONFIG_PATH = pathlib.Path("configs/conf_polaris_gem.lanenet.yaml")
    WEIGHTS_PATH = pathlib.Path("lanenet_weights/tusimple_lanenet.ckpt")
    IMAGES_DIR = pathlib.Path("test/gem_test_images")

    def setUp(self) -> None:
        if not self.CONFIG_PATH.is_file():
            raise FileNotFoundError("Config path %s does not exist or is not a file" % self.CONFIG_PATH.absolute())
        if not self.WEIGHTS_PATH.parent.exists():
            raise FileNotFoundError("Weights path %s not exists" % self.WEIGHTS_PATH.absolute())
        if not self.IMAGES_DIR.is_dir():
            raise FileNotFoundError("Folder for video captures %s does not exist or is not a folder" %
                                    self.IMAGES_DIR.absolute())

        self.nnet = LaneNetWLineFit(self.CONFIG_PATH.as_posix(), self.WEIGHTS_PATH.as_posix(), debug=True)
        # Block and show the image.
        # Set to true if you are running tests manually and want to see the plotted images
        self.plt_show_block = True

    def test_detect(self):
        for img_path in self.IMAGES_DIR.iterdir():
            if not img_path.is_file():
                print("Image %s is not a file. Skip." % img_path.as_posix())
                continue
            img = cv2.imread(img_path.as_posix())
            img = cv2.resize(img, (960, 540), interpolation=cv2.INTER_LINEAR)
            line_seq, annotated_image = self.nnet.detect(img)

            left_line, right_line = None, None
            left_line_y_diff, right_line_y_diff = -np.inf, np.inf
            # Extrapolate the edge line and calculate the difference to base_footprint
            for edge_line in line_seq:
                y_diff = edge_line(0.0) - 0.0
                if left_line_y_diff < y_diff < 0:
                    left_line = edge_line
                    left_line_y_diff = y_diff
                elif 0 < y_diff < right_line_y_diff:
                    right_line = edge_line
                    right_line_y_diff = y_diff
                elif y_diff == 0:
                    # TODO raise warning
                    continue

            if left_line is None or right_line is None:
                # TODO handle error when detected lanes < 2
                continue
            center_line = (left_line + right_line) / 2

            yaw_err = np.arctan(center_line.convert().coef[1])
            label_str = 'Heading error: %.2f deg' % np.rad2deg(yaw_err)
            annotated_image = cv2.putText(annotated_image, label_str, (30, 40), 0, 1, (255, 255, 255), 2, cv2.LINE_AA)

            # base_footprint or base_link is the origin (0.0, 0.0)
            y_diff = center_line(0.0) - 0.0
            offset_base_footprint = y_diff * np.cos(yaw_err)
            label_str = 'Offset from vehicle rear axle (base_footprint): %.2f m' % offset_base_footprint
            annotated_image = cv2.putText(annotated_image, label_str, (30, 70), 0, 1, (255, 255, 255), 2, cv2.LINE_AA)

            # front axle is at (1.75, 0.0)
            y_diff = center_line(1.75) - 0.0
            offset_front_axle = y_diff * np.cos(yaw_err)
            label_str = 'Offset from vehicle front axle: %.2f m' % offset_front_axle
            annotated_image = cv2.putText(annotated_image, label_str, (30, 100), 0, 1, (255, 255, 255), 2, cv2.LINE_AA)

            plt_mask_img = cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGB)
            plt.imshow(plt_mask_img)
        plt.show(block=self.plt_show_block)

    def tearDown(self) -> None:
        self.nnet.close()
