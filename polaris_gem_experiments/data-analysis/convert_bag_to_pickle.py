#!/usr/bin/env python3
import pickle
from typing import Any, NamedTuple, Tuple

from scipy.spatial.transform import Rotation

from genpy import TVal
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rosbag import Bag

from gem_lane_detect_msgs.msg import SimpleLaneStamped

StampedData = NamedTuple("StampedData", [('stamp', int), ('data', Tuple[float, ...])])

ODOM_TOPIC = "base_footprint/odom"
LANE_TOPIC = "estimated_lane"


def quat_to_yaw(quat: Quaternion) -> float:
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    yaw, pitch, roll = Rotation.from_quat((x, y, z, w)).as_euler("ZYX")  # Follow REP-103
    return yaw


def odom_to_state(msg: Odometry) -> Tuple[float, float, float]:
    yaw = quat_to_yaw(msg.pose.pose.orientation)
    return msg.pose.pose.position.x, msg.pose.pose.position.y, yaw


def lane_to_percept(msg: SimpleLaneStamped) -> Tuple[float, float]:
    return msg.lane.offset, msg.lane.yaw_err


def convert_bag_messages(bag: Bag, topic_to_cb):
    for topic, msg, t in bag.read_messages(topics=topic_to_cb.keys()):
        if hasattr(msg, "header"):
            t = msg.header.stamp  # type: TVal  # Use sender's timestamp if available

        t_nsec = t.to_nsec()

        if topic in topic_to_cb:
            data = topic_to_cb[topic](msg)
            yield topic, StampedData(stamp=t_nsec, data=data)
        else:
            continue  # Ignore other topics


def process_bag_file(file_name: str, topic_to_cb):
    with Bag(file_name) as bag:
        converted_iter = convert_bag_messages(bag, topic_to_cb)

        ret_dist_trace = {key: [] for key in topic_to_cb.keys()}
        for topic, stamped_data in converted_iter:
            ret_dist_trace[topic].append(tuple(stamped_data))

    # Make sure each list is sorted by time stamp
    for lst in ret_dist_trace.values():  # type: list
        lst.sort(key=lambda v: v[0])

    stamped_state_list = ret_dist_trace[ODOM_TOPIC]
    if len(ret_dist_trace.get(LANE_TOPIC, ())) > 0:
        stamped_percept_list = ret_dist_trace[LANE_TOPIC]
    else:
        stamped_percept_list = []

    return file_name, stamped_state_list, stamped_percept_list


def main(argv: Any) -> None:
    TOPIC_TO_CB = {
        ODOM_TOPIC: odom_to_state,
        LANE_TOPIC: lane_to_percept
    }

    file_samples_iter = (process_bag_file(bag_file_name, TOPIC_TO_CB)
                         for bag_file_name in argv.bag_file)

    for file_name, stamped_states, stamped_percepts in file_samples_iter:
        with open("%s.pickle" % file_name, 'wb') as f:
            data_w_header = {
                "fields": {"state": ('x', 'y', 'yaw'), "percept": ('cte', 'psi')},
                "stamped_states": stamped_states,
                "stamped_percepts": stamped_percepts
            }
            pickle.dump(data_w_header, f)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('bag_file', nargs='+', type=str)
    main(parser.parse_args())
