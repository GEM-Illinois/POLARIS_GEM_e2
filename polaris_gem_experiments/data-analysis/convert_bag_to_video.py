#!/usr/bin/env python3
"""
Convert image messages to a video for easier visualization and debugging.
Note that the timestamps and other messages in ROS bags are discarded.
"""

import pathlib
import time

import numpy as np
import av

import cv_bridge
import rospy
import rosbag


CAM_IMG_TOPIC = "front_single_camera/image_raw/compressed"


def process_bag_file(in_bag: rosbag.Bag,
                     sec_start=0.0, sec_stop=np.inf):
    t_start = rospy.Time.from_sec(sec_start) if np.isfinite(sec_start) else None
    t_stop = rospy.Time.from_sec(sec_stop) if np.isfinite(sec_stop) else None

    bridge = cv_bridge.CvBridge()

    progress_stamp = t_start
    stamp_itvl = rospy.Duration(secs=5)
    for topic, msg, t in in_bag.read_messages(start_time=t_start, end_time=t_stop):
        if hasattr(msg, "header"):
            t = msg.header.stamp  # Use sender's timestamp if available

        if topic == CAM_IMG_TOPIC:
            cv_image = bridge.compressed_imgmsg_to_cv2(msg, "rgb8")
            yield cv_image

        if t > progress_stamp + stamp_itvl:
            print("Finished processing "
                  "%.1f to %.1f second" % (progress_stamp.to_sec(), (progress_stamp + stamp_itvl).to_sec()))
            progress_stamp += stamp_itvl


def main(argv) -> None:
    sec_start = argv.start  # second
    sec_stop = argv.stop  # second

    out_name_modifier = ".clip-%s-%s" % (str(sec_start).zfill(3), str(sec_stop).zfill(3))

    in_bag_name = argv.bag_file
    in_bag_path = pathlib.Path(in_bag_name)
    out_video_name = argv.output if argv.output \
        else in_bag_path.with_name(in_bag_path.stem + out_name_modifier + ".mp4").as_posix()

    print("Saving to new video file at %s" % out_video_name)

    begin = time.time()

    with rosbag.Bag(in_bag_name) as in_bag, av.open(out_video_name, mode='w') as container:
        stream = container.add_stream('hevc', rate=30)
        stream.width = 960
        stream.height = 540
        stream.pix_fmt = 'gbrp'
        # stream.codec_context.options['crf'] = '0'  # lossless compression
        stream.codec_context.options['crf'] = '18'  # visually lossless
        for cv_image in process_bag_file(in_bag, sec_start, sec_stop):
            frame = av.VideoFrame.from_ndarray(cv_image, format='rgb24')

            for packet in stream.encode(frame):
                container.mux(packet)
        # Flush stream
        for packet in stream.encode():
            container.mux(packet)

    print("Converted %d images" % stream.frames)
    print("Total Time usage: %.2f seconds" % (time.time() - begin))


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Convert image messages in a ROS bag to a video for visualization and debugging.")
    parser.add_argument('bag_file', type=str)
    parser.add_argument(
        '-o', '--output', type=str, default="",
        help="Output video file")
    parser.add_argument(
        '--start', nargs='?', type=float, default=0.0, metavar='START',
        help="Processing only those messages with timestamp >= %(metavar)s second. (default: %(default)s)")
    parser.add_argument(
        '--stop', nargs='?', type=float, default=float('inf'), metavar='STOP',
        help="Processing only those messages with timestamp < %(metavar)s second. (default: %(default)s)")
    main(parser.parse_args())
