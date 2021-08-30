import cv2
import numpy as np
import tensorflow as tf
from lanenet.lanenet_model import lanenet
from lanenet.parse_config_utils import Config
from gem_lanenet.line_fit import Line, line_fit_pipeline


class LaneNetWLineFit:
    def __init__(self, config_path: str, weights_path: str,
                 use_gpu: bool = True, window_size: int = 5) -> None:
        if tf.compat.v1.executing_eagerly():
            raise RuntimeError(
                "Currently only support running TensorFlow with eager execution disabled."
                "For example, call `tensorflow.compat.v1.disable_eager_execution()` to disable eager execution.")

        self._cfg = Config(config_path=config_path)
        self._init_lanenet(weights_path, use_gpu)

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
    def _preprocess(img):
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

    def _inference_net(self, img, original_img):
        binary_seg_image, instance_seg_image = self._sess.run(
            [self.binary_seg_ret, self.instance_seg_ret],
            feed_dict={self.input_tensor: [img]}
        )
        binary_seg_result = np.reshape(binary_seg_image[0], (256, 512)).astype(np.float)

        for i in range(self._cfg.MODEL.EMBEDDING_FEATS_DIMS):
            instance_seg_image[0][:, :, i] = self._minmax_scale(instance_seg_image[0][:, :, i])
        embedding_image = np.array(instance_seg_image[0], np.uint8)

        # Edge detection on embedding instance image
        edges = np.array(cv2.Canny(embedding_image, 100, 200), dtype=np.float)
        fusion = cv2.addWeighted(binary_seg_result, 1, edges, 1, 0)

        # Fusion morp
        kernel = np.ones((3, 3), np.uint8)
        # dilation = cv2.dilate(fusion,kernel,iterations = 1)
        closing = cv2.morphologyEx(fusion, cv2.MORPH_CLOSE, kernel)
        closing = closing.astype(np.float32)
        orig = cv2.resize(original_img, (1280, 720), interpolation=cv2.INTER_LINEAR)
        binary_seg_image = binary_seg_image.astype(np.float32)
        img = cv2.resize(img, (1280, 720), interpolation=cv2.INTER_LINEAR)
        # combined detection added for test
        fit_ret = line_fit_pipeline(closing, orig, self.detected, self.left_line, self.right_line)

        mask_image = fit_ret['combined_img']
        offset = fit_ret['lane_offset']
        avg_cur = fit_ret['avg_curve']
        self.detected = fit_ret['detected']
        self.left_line = fit_ret['left_line']
        self.right_line = fit_ret['right_line']

        return mask_image, offset, avg_cur

    def detect(self, img):
        resized_img = self._preprocess(img)
        mask_image, offset, curvature = self._inference_net(resized_img, img)
        return mask_image, offset, curvature

    def close(self) -> None:
        self._sess.close()  # TODO add support for `with` statement using context manager
