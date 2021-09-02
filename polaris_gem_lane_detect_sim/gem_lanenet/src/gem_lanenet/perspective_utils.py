import ast
import numpy as np
import cv2
import matplotlib.pyplot as plt
from math import sqrt

#
# Pixel coordinates for the corners of a stop sign, 31.5 centimeters
# to a side, that I put on the floor in front of the Polaris
# GEM e2 and photographed with the Mako camera (ROS topic
# /mako_1/mako_1/image_raw):
#
#     tl1 ------ tr1
#      /          \
#     /            \
#    tl2          tr2
#     |   S T O P  |
#    bl2          br2
#     \            /
#      \          /
#     bl1 ------ br1
#
# I call corners tl1, tr1, br1, bl1, "primary," and corners
# tl2, tr2, br2, bl2, "secondary."
#

# Length of a side in meters.
STOP_SIGN_SIDE = 31.5 / 100
# Width of the sign in meters.
STOP_SIGN_WIDTH = STOP_SIGN_SIDE * (1 + sqrt(2))
# Scaling factor for bird's-eye view
SCALING_FACTOR = .75


def perp(u):
    """
    Return a 2-vector perpendicular to the 2-vector `u`.

    Proof: let . be the dot product.  Observe that perp(u) . u == 0.
    """
    return np.float32([u[1], -u[0]])


class Perspective:
    def __init__(self, corners_file='perspective/wide-perspective'):
        with open(corners_file, 'r') as f:
            stop_sign_corners = f.read()
            _corners = ast.literal_eval(stop_sign_corners)
            [self.tl1, self.tr1, self.tr2, self.br2, self.br1, self.bl1,
             self.bl2, self.tl2] = _corners
            self.corners = np.array(_corners, dtype=np.float32)
        # Meters per x-pixel.
        self.xm_per_pix = STOP_SIGN_WIDTH / (np.linalg.norm(np.subtract(self.br2, self.bl2)) * SCALING_FACTOR)
        # Meters per y-pixel.
        self.ym_per_pix = self.xm_per_pix
        # self.node = node

    def birdeye(self, img, verbose=False):
        """
        Apply perspective transform to input frame to get the bird's eye view.
        :param img: input color frame
        :param verbose: if True, show the transformation result
        :return: warped image, and both forward and backward transformation matrices
        """

        h, w = img.shape[:2]

        # `cross` is the vector bl2 -> br2.  We use the length of `cross`
        # as the height of the stop sign in the bird's-eye view.
        cross = np.subtract(self.br2, self.bl2)

        def center(pts):
            return sum(pts) / len(pts)

        # `xofs` is the vector that translates the stop sign to the
        # center of the frame.
        frame_mid_x = w / 2
        stop_sign_mid_x = center(self.corners)[0]
        xofs = np.float32([frame_mid_x - stop_sign_mid_x, 0])

        # Find the transformation matrix `M` that takes the foreshortened,
        # horizontally-offset primary corners, `src`, into the primary corners
        # of a roughly equilateral, horizontally-centered octagon on the
        # bird's-eye view.
        src = np.array([self.br1, self.bl1, self.tl1, self.tr1], dtype=np.float32)

        def horizontally_center(p):
            return np.add(xofs, p)

        def scale_around_center(pts):
            c = center(pts)
            return list(map(lambda p: np.add(c, np.subtract(p, c) * SCALING_FACTOR),
                            pts))

        dst = np.float32(scale_around_center(list(map(horizontally_center,
                                                      [self.br1,
                                                       self.bl1,
                                                       np.add(self.bl1, perp(cross)),
                                                       np.add(self.br1, perp(cross))]))))

        M = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, src)

        warped = cv2.warpPerspective(img, M, (w, h), flags=cv2.INTER_LINEAR)

        if verbose:
            f, axarray = plt.subplots(1, 2)
            f.set_facecolor('white')
            axarray[0].set_title('Before perspective transform')
            axarray[0].imshow(img, cmap='gray')
            for point in src:
                axarray[0].plot(*point, '.')
            axarray[1].set_title('After perspective transform')
            axarray[1].imshow(warped, cmap='gray')
            for point in dst:
                axarray[1].plot(*point, '.')
            for axis in axarray:
                axis.set_axis_off()
            plt.show()

        def diag_image():
            return np.dstack((warped, warped, warped)) * 255

        # self.node.display("birdeye", diag_image)
        return warped, M, Minv


def my_bird(img):
    # TODO need improved and calibrated
    src = np.array([[350, 520], [1000, 520], [1200, 700], [245, 700]], dtype=np.float32)
    # dst = np.array([[200, 20], [1050, 0], [1100, 720], [245, 720]], dtype=np.float32)
    dst = np.array([[50, 20], [900, 0], [950, 720], [95, 720]], dtype=np.float32)

    M = cv2.getPerspectiveTransform(src, dst)  # The transformation matrix
    Minv = cv2.getPerspectiveTransform(dst, src)  # Inverse transformation

    # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    warped_img = cv2.warpPerspective(img, M, (1280, 720))  # Image warping

    # Uncomment if lane gets wild 
    # x points diff 2.3m
    # y top to bottom 2m
    cv2.circle(warped_img, (240, 700), 14, (0, 255, 0), -1)
    cv2.circle(warped_img, (890, 700), 14, (0, 255, 0), -1)
    cv2.circle(warped_img, (240, 660), 14, (0, 255, 0), -1)
    cv2.circle(warped_img, (890, 660), 14, (0, 255, 0), -1)

    return warped_img, M, Minv
