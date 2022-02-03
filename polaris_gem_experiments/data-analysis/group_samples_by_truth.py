#!/usr/bin/env python3
import math
import pickle
from typing import Any, Mapping, Tuple
import warnings

import yaml

import numpy as np
from scipy.spatial import KDTree

HEADING_THRES = 3 * 10 ** -4
DISTANCE_THRES = 2 * 10 ** -4

# Parameter specific to our Gazebo world
SCENE_SEP = 30.0  # meter
PLOT_NUM = 3


def is_close(l1: Tuple[float, float], l2: Tuple[float, float]) -> bool:
    return abs(l1[0] - l2[0]) <= HEADING_THRES and abs(l1[1] - l2[1]) <= DISTANCE_THRES


def state_to_truth(state: Tuple[float, float, float]) -> Tuple[float, float]:
    x, y, yaw = state
    # Normalize y given the three road scenes are translational symmetric
    y = (y + SCENE_SEP / 2) % SCENE_SEP - SCENE_SEP / 2
    assert abs(y) < SCENE_SEP / 2
    return -y, -yaw


def get_closest_truth(predefined_kd_tree: KDTree, state: Tuple[float, float, float]) -> Tuple[float, float]:
    recorded_truth = state_to_truth(state)
    dist, i = predefined_kd_tree.query(recorded_truth)
    if is_close(predefined_kd_tree.data[i], recorded_truth):
        return predefined_kd_tree.data[i]
    print("Warning: recorded state %s is not close to any predefined ground truths" % str(state))
    return recorded_truth


def validate_fields(truth_fields: Mapping[str, Tuple[str, ...]],
                    sample_fields: Mapping[str, Tuple[str, ...]]) -> bool:
    return tuple(sample_fields.get("state")) == ('x', 'y', 'yaw') \
        and tuple(sample_fields.get("percept")) == ('cte', 'phi') \
        and tuple(truth_fields.get("truth")) == ('cte', "phi")


def merge_state_percept_as_sample(sorted_state_list, sorted_percept_list):
    """ Heuristically merge states and percepts as samples according to timestamps.
        Given (t[i], state[i]), (t[i+1], state[i+1]), and t[i] <= t[j] < t[i+1],
        we assume a percept (t[j], percept[j]) is for state[i].
        Both lists should be sorted by the timestamp.
    """
    if len(sorted_state_list) == 0:
        warnings.warn("No states")
        return
    if len(sorted_percept_list) == 0:
        warnings.warn("No percepts")
        return
    if not all(v0[0] < v1[0] for v0, v1 in zip(sorted_state_list[:-1],
                                               sorted_state_list[1:])):
        raise ValueError("The timestamps for states are not strictly increasing")
    if not all(v0[0] < v1[0] for v0, v1 in zip(sorted_percept_list[:-1],
                                               sorted_percept_list[1:])):
        raise ValueError("The timestamps for percepts are not strictly increasing")

    state_iter = iter(sorted_state_list)
    curr_t_i, curr_state = next(state_iter)
    percept_iter = iter(sorted_percept_list)
    curr_t_j, curr_percept = next(percept_iter)

    # Skip percepts before first state
    while curr_t_j < curr_t_i:
        curr_t_j, curr_percept = next(percept_iter, (None, None))
        if curr_t_j is None:
            warnings.warn("All percepts are recorded before any state")
            return
    # Merge state list and percept list
    next_t_i, next_state = next(state_iter, (None, None))
    while next_t_i is not None:
        assert curr_t_i <= curr_t_j
        if curr_t_j < next_t_i:  # Advance to next percept
            yield curr_state, curr_percept
            curr_t_j, curr_percept = next(percept_iter, (None, None))
            if curr_t_j is None:
                return
        else:  # Advance to next state
            curr_t_i, curr_state = next_t_i, next_state
            next_t_i, next_state = next(state_iter, (None, None))
    # All percepts after last state
    for t_j, percept in percept_iter:
        assert t_j >= curr_t_i
        yield curr_state, percept


def main(argv: Any) -> None:
    yaml_data = yaml.safe_load(argv.predefined_truths)
    predefined = yaml_data["truth_list"]
    truth_fields = yaml_data["fields"]
    predefined_kd_tree = KDTree(np.array(predefined))

    truth_samples = []
    for pickle_file_io in argv.pickle_file:
        pkl_data = pickle.load(pickle_file_io)
        if not validate_fields(truth_fields, pkl_data.get("fields", {})):
            warnings.warn("Skip pickle file %s because the fields do not match" % pickle_file_io.name)
            continue
        stamped_states = pkl_data["stamped_states"]
        stamped_percepts = pkl_data["stamped_percepts"]

        for state, percept in merge_state_percept_as_sample(stamped_states, stamped_percepts):
            truth = get_closest_truth(predefined_kd_tree, state)
            sample = state + percept  # Concatenate tuples
            # Ensure using built-in Python data type
            truth = tuple(float(v) for v in truth)
            sample = tuple(float(v) for v in sample)
            if len(truth_samples) > 0 and is_close(truth_samples[-1][0], truth):
                truth_samples[-1][1].append(sample)  # Group with previous truth
            else:
                truth_samples.append((truth, [sample]))

    # Filter those truths without enough samples
    truth_samples = [entry for entry in truth_samples if len(entry[1]) > 10]

    print("Number of NaN samples: %d" % sum(len([s for s in raw_samples if any(math.isnan(field) for field in s)])
                                            for t, raw_samples in truth_samples))
    # Filter NaN
    if argv.no_nan:
        truth_samples = [(t, [s for s in raw_samples if not any(math.isnan(field) for field in s)])
                         for t, raw_samples in truth_samples]

    for truth, samples in truth_samples:
        print("Ground truth: %s; #Samples: %d" % (str(truth), len(samples)))

    print("Total Number of Ground Truths: %d" % len(truth_samples))
    if len(truth_samples) != len(predefined):
        print("Total Number of Ground Truths  %d is not equal to expected number %d"
              % (len(truth_samples), len(predefined)))
        print("Please double check if the predefined ground truths are specified correctly.")
    if argv.output is not None:
        print("Save to %s" % argv.output.name)
        data = {
            "truth_config": argv.predefined_truths.name,
            "fields": {"truth": ("cte", "phi"), "samples": ("x", "y", "yaw", "cte", "phi")},
            "truth_samples": truth_samples
        }
        if "distribution" in yaml_data:
            data["truth_distribution"] = yaml_data["distribution"]
        pickle.dump(data, argv.output)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('predefined_truths', type=argparse.FileType('rb'))
    parser.add_argument('pickle_file', nargs='+', type=argparse.FileType('rb'))
    parser.add_argument('--no-nan', type=bool, default=False, help="Filter NaN values. (default: %(default)s)")
    parser.add_argument('-o', '--output', type=argparse.FileType('wb'), help="Save output as a pickle file")
    main(parser.parse_args())
