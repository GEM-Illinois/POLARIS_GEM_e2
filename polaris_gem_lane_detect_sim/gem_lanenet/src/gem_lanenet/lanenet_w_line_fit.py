from typing import Dict, Optional, Tuple

import cv2
import numpy as np
from numpy.polynomial.polynomial import Polynomial
import tensorflow as tf
from lanenet.lanenet_model import lanenet, lanenet_postprocess
from lanenet.parse_config_utils import Config
from gem_lanenet.line_fit import Line

WARPED_IMG_W = 16*61  # pixels
WARPED_IMG_H = 9*61  # pixels
WARPED_IMG_SIZE = (WARPED_IMG_W, WARPED_IMG_H)
LOOKAHEAD_PIXELS = WARPED_IMG_H // 2
METER_PER_PIXEL = 0.03  # each pixel is about 0.03 m in the warped image

COLOR_DICT = {
    0: (0, 255, 0),
    1: (0, 255, 255),
    2: (240, 248, 255),
    3: (227, 207, 87),
}

SRC_DST_MAP = {
    (412, 329): (426, 60),
    (401, 338): (426, 121),
    (386, 351): (426, 182),
    (364, 370): (426, 243),
    (329, 400): (426, 304),
    (263, 457): (426, 365),
    (477, 329): (487, 60),
    (477, 338): (487, 121),
    (477, 351): (487, 182),
    (476, 370): (487, 243),
    (474, 400): (487, 304),
    (472, 457): (487, 365),
    (543, 329): (548, 60),
    (553, 338): (548, 121),
    (567, 351): (548, 182),
    (588, 370): (548, 243),
    (620, 400): (548, 304),
    (681, 457): (548, 365),
    (609, 329): (609, 60),
    (630, 338): (609, 121),
    (658, 351): (609, 182),
    (699, 370): (609, 243),
    (766, 400): (609, 304),
    (891, 457): (609, 365),
    (675, 329): (670, 60),
    (706, 338): (670, 121),
    (749, 351): (670, 182),
    (811, 370): (670, 243),
    (912, 400): (670, 304),
    (1100, 457): (670, 365)}


def get_perspective_transform(src_dst_map: Dict[Tuple[int, int], Tuple[int, int]]) \
        -> Tuple[np.ndarray, np.ndarray]:
    """ Compute perspective transformation matrices for converting images to
        bird's eye view using homography.

    Parameters
    ----------
    src_dst_map
        A dictionary mapping a source pixel in source image to a destination
        pixel in warped image

    Returns
    -------
    matrix, matrix_inv
        Perspective transformation matrix and its inverse
    """
    src_pts, dst_pts = list(zip(*src_dst_map.items()))
    src_pts_arr = np.float32(src_pts)
    dst_pts_arr = np.float32(dst_pts)
    matrix, mask = cv2.findHomography(src_pts_arr, dst_pts_arr, cv2.RANSAC, 5.0)
    matrix_inv, mask_inv = cv2.findHomography(dst_pts_arr, src_pts_arr, cv2.RANSAC, 5.0)
    return matrix, matrix_inv


def _find_pixels_in_window(pixel_arr: np.ndarray,
                           curr_pixel: Tuple[int, int],
                           img_size: Tuple[int, int],
                           window_size: Tuple[int, int]):
    u_curr, v_curr = curr_pixel
    img_w, img_h = img_size
    if not(0 <= u_curr < img_w and 0 <= v_curr < img_h):
        raise ValueError("Given curr_pixel %s is not in image size %s"
                         % (curr_pixel, img_size))
    window_w, window_h = window_size

    # Ensure within source image bound
    window_u_lo, window_u_hi = u_curr, min(u_curr + window_w, img_w)
    window_v_lo, window_v_hi = v_curr, min(v_curr + window_h, img_h)

    pixel_filter = \
        (window_u_lo <= pixel_arr[:, 0]) & (pixel_arr[:, 0] < window_u_hi) \
        & (window_v_lo <= pixel_arr[:, 1]) & (pixel_arr[:, 1] < window_v_hi)
    return pixel_arr[pixel_filter]


def _find_lane_edge_pixels(nonzero_pixels: np.ndarray,
                           base_vertical_line: int,
                           img_size: Tuple[int, int],
                           nwindows: int = 9,
                           min_pixels: int = WARPED_IMG_W//24) \
        -> np.ndarray:
    """ Find the nonzero pixels for one lane

    Parameters
    ----------
    nonzero_pixels
        An array of pixels that have nonzero value in the source image
    base_vertical_line
        The vertical line to start searching for lane marking pixels
    img_size
        Size of the source image in (width, height)

    Returns
    -------
    lane_pixel_arr
        Array of pixels for the lane close to base_vertical_line

    """
    img_w, img_h = img_size
    window_w, window_h = img_w // 12, img_h // nwindows  # FIXME tune the window width
    u_base = base_vertical_line
    if not(0 <= u_base < img_w):
        raise ValueError("Given base vertical line %s is not in image size %s"
                         % (u_base, img_size))

    lane_pixels = []
    u_curr = max(u_base - window_w//2, 0)
    # Start from img_h to search from bottom-up
    for v_curr in img_h - window_h*np.arange(1, nwindows+1, dtype=np.int):
        good_pixel_arr = _find_pixels_in_window(
            pixel_arr=nonzero_pixels,
            curr_pixel=(u_curr, v_curr),
            img_size=img_size,
            window_size=(window_w, window_h))

        if len(good_pixel_arr) > min_pixels:
            new_u_base = int(np.mean(good_pixel_arr[:, 0]))
            u_curr = max(new_u_base - window_w//2, 0)

        # Append these pixels to the list
        lane_pixels.extend(good_pixel_arr)

    # Ensure the shape must be Nx2 even when N=0
    lane_pixel_arr = np.array(lane_pixels, dtype=np.int32).reshape((-1, 2))
    return lane_pixel_arr


def _find_current_lane_pixels(binary_birdeye_img) -> Tuple[np.ndarray, np.ndarray]:
    """ Find the pixels of the two lane edges for the current lane on the
        binary bird's eye view image

    Parameters
    ----------
    binary_birdeye_img: np.ndarray
        Input binary bird's eye view image

    Returns
    -------
    left_lane_pixels, right_lane_pixels
        Two numpy arrays for left and right lane edges. Each represents an array of pixels.

    Notes
    -----
    Pixel is a pair of (u, v) where 0<=u<img_w and 0<=v<img_h following
    OpenCV convention.
    Beware that u is column and v is row for Numpy arrays.

    References
    ----------
    https://automaticaddison.com/the-ultimate-guide-to-real-time-lane-detection-using-opencv/
    """
    img_size = (binary_birdeye_img.shape[1], binary_birdeye_img.shape[0])

    # Take a histogram of the bottom half of the image
    bot_half_img = binary_birdeye_img[binary_birdeye_img.shape[0]//2:, :]
    histogram = np.sum(bot_half_img, axis=0)

    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = histogram.shape[0] // 2
    left_u_base = np.argmax(histogram[:midpoint])
    right_u_base = np.argmax(histogram[midpoint:]) + midpoint

    # Identify (u, v) of all nonzero pixels in the image
    nonzero_vs, nonzero_us = binary_birdeye_img.nonzero()
    nonzero_pixels = np.vstack((nonzero_us, nonzero_vs)).transpose()

    # Find pixels for the two lanes
    # NOTE an alternative method is to cluster the pixels with additional info
    # from instance segmentation done by LaneNet
    left_lane_pixels = _find_lane_edge_pixels(nonzero_pixels, int(left_u_base), img_size)
    right_lane_pixels = _find_lane_edge_pixels(nonzero_pixels, int(right_u_base), img_size)
    return left_lane_pixels, right_lane_pixels


def _find_all_lanes_pixels(binary_birdeye_img,
                           min_h: int = WARPED_IMG_H//9,
                           min_pixels: int = WARPED_IMG_W//24) -> Tuple[np.ndarray, ...]:
    num_labels, labeled_img, stats, centroids = \
        cv2.connectedComponentsWithStats(binary_birdeye_img, connectivity=8, ltype=cv2.CV_16U)

    ret = []
    for label in range(1, num_labels):
        # Exclude components with too small height or area
        if stats[label, cv2.CC_STAT_HEIGHT] < min_h \
                or stats[label, cv2.CC_STAT_AREA] < min_pixels:
            continue
        v_arr, u_arr = np.nonzero(labeled_img == label)
        pixels = np.vstack((u_arr, v_arr)).transpose()
        ret.append(pixels)
    return tuple(ret)


def _convert_poly_pixel_to_pos(line: Polynomial) -> Polynomial:
    c0, c1 = line.convert().coef[:2]
    # Transform the line with
    #  x = -METER_PER_PIXEL*(v - WARPED_IMG_H)
    #  y = -METER_PER_PIXEL*(u - WARPED_IMG_W//2)
    #  where x-axis points toward vehicle's forward direction and y-axis to the left
    # Given u = c0 + c1*v, the transformed line is:
    #  y = METER_PER_PIXEL*(WARPED_IMG_W//2 - (c0 + c1*WARPED_IMG_H)) + c1*x
    c0_p = METER_PER_PIXEL * (WARPED_IMG_W // 2 - (c0 + c1 * WARPED_IMG_H))
    c1_p = c1
    line_base_footprint = Polynomial([c0_p, c1_p])
    return line_base_footprint


class LaneNetWLineFit:
    M, M_INV = get_perspective_transform(SRC_DST_MAP)

    def __init__(self, config_path: str, weights_path: str,
                 use_gpu: bool = True, debug: bool = False, window_size: int = 5) -> None:
        if tf.compat.v1.executing_eagerly():
            raise RuntimeError(
                "Currently only support running TensorFlow with eager execution disabled."
                "For example, call `tensorflow.compat.v1.disable_eager_execution()` to disable eager execution.")

        self._cfg = Config(config_path=config_path)
        self._init_lanenet(weights_path, use_gpu)
        self._cluster = lanenet_postprocess._LaneNetCluster(cfg=self._cfg)
        self._debug = debug

        self.detected = False
        self.left_line = Line(n=window_size)
        self.right_line = Line(n=window_size)

    def _init_lanenet(self, weights_path: str, use_gpu: bool) -> None:
        """ Initialize the TensorFlow model """
        self.input_tensor = tf.compat.v1.placeholder(dtype=tf.float32, shape=[1, 256, 512, 3], name='input_tensor')
        phase_tensor = tf.constant('test', tf.string)
        net = lanenet.LaneNet(phase=phase_tensor, cfg=self._cfg)
        self.binary_seg_ret, self.instance_seg_ret = net.inference(input_tensor=self.input_tensor, name='LaneNet')

        # Set sess configuration
        if use_gpu:
            sess_config = tf.compat.v1.ConfigProto(device_count={'GPU': 1})
        else:
            sess_config = tf.compat.v1.ConfigProto(device_count={'CPU': 2})
        sess_config.gpu_options.per_process_gpu_memory_fraction = self._cfg.GPU.GPU_MEMORY_FRACTION
        sess_config.gpu_options.allow_growth = self._cfg.GPU.TF_ALLOW_GROWTH
        sess_config.gpu_options.allocator_type = 'BFC'

        self._sess = tf.compat.v1.Session(config=sess_config)
        saver = tf.compat.v1.train.Saver()
        saver.restore(sess=self._sess, save_path=weights_path)

    @staticmethod
    def _lanenet_preprocess(img):
        image = cv2.resize(img, (512, 256), interpolation=cv2.INTER_LINEAR)
        image = image / 127.5 - 1.0
        return image

    @staticmethod
    def _minmax_scale(input_arr):
        """
        :param input_arr:
        :return:
        """
        min_val = np.min(input_arr)
        max_val = np.max(input_arr)
        output_arr = (input_arr - min_val) * 255.0 / (max_val - min_val)
        return output_arr

    @staticmethod
    def _lanenet_postprocess(binary_seg_result, instance_seg_result, min_area_threshold=100):
        # convert binary_seg_result to 8-bit
        binary_seg_result = np.array(binary_seg_result * 255, dtype=np.uint8)
        # Use closing morphology. Closing small holes inside the foreground objects
        morphological_ret = lanenet_postprocess._morphological_process(binary_seg_result)

        # Use connected components to remove too small components
        connect_components_analysis_ret = lanenet_postprocess._connect_components_analysis(image=morphological_ret)

        labels = connect_components_analysis_ret[1]
        stats = connect_components_analysis_ret[2]
        for index, stat in enumerate(stats):
            if stat[cv2.CC_STAT_AREA] <= min_area_threshold:
                idx = np.where(labels == index)
                morphological_ret[idx] = 0

        return morphological_ret

    def _lanenet_pipeline(self, src_img):
        resized_img = self._lanenet_preprocess(src_img)
        binary_seg_image, instance_seg_image = self._sess.run(
            [self.binary_seg_ret, self.instance_seg_ret],
            feed_dict={self.input_tensor: [resized_img]}
        )
        # Eliminate redundant dimension which has only one entry
        binary_seg_result = np.squeeze(binary_seg_image)
        instance_seg_result = np.squeeze(instance_seg_image)

        binary_seg_result = self._lanenet_postprocess(binary_seg_result, instance_seg_result)

        for i in range(self._cfg.MODEL.EMBEDDING_FEATS_DIMS):
            instance_seg_result[:, :, i] = self._minmax_scale(instance_seg_result[:, :, i])
        embedding_image = np.array(instance_seg_result, np.uint8)

        # Resize back to the same size as source image
        src_img_size = (src_img.shape[1], src_img.shape[0])
        binary_seg_result = cv2.resize(binary_seg_result, src_img_size, interpolation=cv2.INTER_LINEAR)
        instance_seg_result = cv2.resize(instance_seg_result, src_img_size, interpolation=cv2.INTER_LINEAR)
        return binary_seg_result, instance_seg_result

    @classmethod
    def _birdeye_view(cls, intermediate_img: np.ndarray, src_img_size: Tuple[int, int]):
        """ Get bird's eye view from input image

            Generate warped image in bird view using cv2.warpPerspective()
        """
        if (intermediate_img.shape[1], intermediate_img.shape[0]) != src_img_size:
            raise ValueError("Transformation to bird's eye assumes that the size of the binary image "
                             "is the same as the source image")

        warped_img = cv2.warpPerspective(intermediate_img, cls.M, WARPED_IMG_SIZE,
                                         flags=cv2.INTER_LINEAR)
        return warped_img

    def _cluster_lane_chords(self, binary_birdeye_img, instance_birdeye_img):
        # FIXME This function is not used for now.
        #  Clustering on the original image size is too slow.
        #  Clustering on shrunk images did not do well and further requires mapping the pixels to original size.
        mask, lane_chords = self._cluster.apply_lane_feats_cluster(binary_birdeye_img, instance_birdeye_img)

        # Enlarge back to original size
        mask = cv2.resize(mask, WARPED_IMG_SIZE, interpolation=cv2.INTER_LINEAR)
        return mask, lane_chords

    def detect(self, src_img) -> Tuple[Optional[Polynomial], np.ndarray]:
        """ Detect and return lane edges

        Parameters
        ----------
        src_img
            Image from camera

        Returns
        -------
        line_sequence
            A sequence of polynomials representing the detected lane edges w.r.t the base_footprint.
            It may include edges of multiple lanes or no lane at all.
            Note that by convention, x-axis is the forward direction and y-axis is to the left
            w.r.t ego vehicle for the resulting center line.
        debug_image:
            An annotated image for debugging.
            If self._debug is False, only the binary bird's eye view is returned.
        """
        src_img_size = (src_img.shape[1], src_img.shape[0])

        binary_seg_result, instance_seg_result = self._lanenet_pipeline(src_img)

        binary_birdeye_img = self._birdeye_view(binary_seg_result, src_img_size)

        edge_pixels_seq = _find_all_lanes_pixels(binary_birdeye_img)
        edge_line_seq = []
        for lane_pixels in edge_pixels_seq:
            # Fit a second order polynomial curve to the pixel coordinates for each lane edge
            # Note that v is the x-axis for fitting to follow the right hand rule
            # TODO fit all lanes to parallel curves instead of fitting curves separately
            u_arr, v_arr = lane_pixels.T
            edge_line = Polynomial.fit(v_arr, u_arr, deg=1, domain=[0, WARPED_IMG_H])  # type: Polynomial
            edge_line_seq.append(edge_line)

        left_line, right_line = None, None
        left_line_u_diff, right_line_u_diff = -WARPED_IMG_W//2, WARPED_IMG_W//2
        # Extrapolate the edge line and find two lanes closest to the lookahead point (W//2, H//2)
        for edge_line in edge_line_seq:
            yaw_err = np.arctan(edge_line.convert().coef[1])
            if abs(yaw_err) > np.pi / 4:
                continue  # Ignore lines with too large yaw error

            u_diff = edge_line(WARPED_IMG_H//2) - WARPED_IMG_W//2
            if left_line_u_diff < u_diff < 0:
                left_line = edge_line
                left_line_u_diff = u_diff
            elif 0 < u_diff < right_line_u_diff:
                right_line = edge_line
                right_line_u_diff = u_diff
            elif u_diff == 0:
                # TODO raise warning
                continue

        if left_line is None or right_line is None:
            center_line = None
        else:
            center_line_pixel = (left_line + right_line) / 2
            center_line = _convert_poly_pixel_to_pos(center_line_pixel)

        if not self._debug:
            return center_line, binary_birdeye_img
        # else:  # For DEBUGGING
        # TODO Check unwarped image is close to binary_seg_result
        colored_birdeye_img = cv2.cvtColor(binary_birdeye_img, cv2.COLOR_GRAY2RGB)

        # Plot vehicle forward direction
        cv2.line(colored_birdeye_img,
                 pt1=(WARPED_IMG_W//2, 0), pt2=(WARPED_IMG_W//2, WARPED_IMG_H),
                 color=(127, 127, 127), thickness=5)

        # Fill detected pixels
        for i, lane_pixels in enumerate(edge_pixels_seq):
            for u, v in lane_pixels:
                colored_birdeye_img[v, u] = np.array(COLOR_DICT.get(i, COLOR_DICT[0]))

        v_arr = np.arange(WARPED_IMG_H, dtype=np.int32)
        poly_line_seq = []
        for edge_line in edge_line_seq:
            u_arr = edge_line(v_arr).astype(np.int32)
            poly_line = np.vstack((u_arr, v_arr), ).T
            poly_line_seq.append(poly_line)

        cv2.polylines(colored_birdeye_img, poly_line_seq, False,
                      color=(0, 0, 255), thickness=5)
        colored_unwarped_img = cv2.warpPerspective(colored_birdeye_img, self.M_INV, src_img_size,
                                                   flags=cv2.INTER_LINEAR)
        layered_image = cv2.addWeighted(src_img, 1, colored_unwarped_img, 0.5, 0)

        # crop margin to aligned the size
        # base_footprint should map from (WARPED_IMG_W/2, WARPED_IMG_H) to (src_w/2, src_h)
        src_w, src_h = src_img_size
        new_w_start = WARPED_IMG_W // 2 - src_w // 2
        new_h_start = WARPED_IMG_H - src_h
        cropped_birdeye_img = colored_birdeye_img[new_h_start:new_h_start + src_h,
                                                  new_w_start:new_w_start + src_w]  # row and column

        # Build final image for debugging
        debug_image = np.vstack((cropped_birdeye_img, layered_image))
        return center_line, debug_image

    def close(self) -> None:
        self._sess.close()  # TODO add support for `with` statement using context manager

