#!/usr/bin/env python3
from bisect import bisect
import math
import pickle
from typing import Any, NamedTuple, Tuple

import yaml

StampedLane = NamedTuple("StampedLane", [('stamp', float), ('heading', float), ('distance', float)])
STAMP_THRES = 50.0  # milli-second
HEADING_THRES = 10 ** -3
DISTANCE_THRES = 10 ** -3


def is_close(l1: Tuple[float, float], l2: Tuple[float, float]) -> bool:
    return abs(l1[0] - l2[0]) <= HEADING_THRES and abs(l1[1] - l2[1]) <= DISTANCE_THRES


def get_closest_truth(predefined, recorded_truth: Tuple[float, float]) -> Tuple[float, float]:
    for defined_heading, defined_distance in predefined:
        if is_close((defined_heading, defined_distance), recorded_truth):
            return defined_heading, defined_distance
    print("Warning: recorded ground truth %s is not close to any predefined ground truths" % str(recorded_truth))
    return recorded_truth


def separate_est_by_truth(sorted_truth_list, sorted_est_list):
    """ Cut the original trace into segments based on the timestamps of ground truth lane.
        Both lists should be sorted by the timestamp.
    """
    if len(sorted_truth_list) == 0:
        print("No ground truth")
        return
    if len(sorted_est_list) == 0:
        print("No estimations")
        return

    keys = [d.stamp for d in sorted_est_list]

    lo = 0
    truth_iter = iter(sorted_truth_list)
    prev_truth = next(truth_iter)
    prev_idx = bisect(keys, prev_truth.stamp, lo=lo)
    for truth in truth_iter:  # type: StampedLane
        idx = bisect(keys, truth.stamp, lo=lo)
        yield prev_truth, sorted_est_list[prev_idx:idx]
        prev_truth, prev_idx = truth, idx
    yield prev_truth, sorted_est_list[prev_idx:]


def concatenate(file_io_seq):
    ret_truth_list, ret_sample_list = [], []
    prev_stamp = 0.0
    prev_filename = ""
    for pickle_file_io in file_io_seq:
        truth_list, sample_list = pickle.load(pickle_file_io)
        assert sample_list[0].stamp > prev_stamp, "timestamps in pickle files is not increasing"
        if sample_list[0].stamp - prev_stamp > STAMP_THRES:
            print("Warning: timestamp gap between pickle file '%s' and '%s' is large than %.1f milli-second" %
                  (prev_filename, pickle_file_io.name, STAMP_THRES))
        ret_truth_list.extend(truth_list)
        ret_sample_list.extend(sample_list)

        prev_stamp = sample_list[-1].stamp
        prev_filename = pickle_file_io.name

    return ret_truth_list, ret_sample_list


def main(argv: Any) -> None:
    predefined = yaml.safe_load(argv.predefined_truths)["truth_list"]

    truth_list, sample_list = concatenate(argv.pickle_file)

    truth_samples = []
    for stamped_truth, stamped_samples in separate_est_by_truth(truth_list, sample_list):
        truth = get_closest_truth(predefined, (stamped_truth.heading, stamped_truth.distance))
        samples = [(est_h, est_d) for (_, est_h, est_d) in stamped_samples]
        if len(truth_samples) > 0 and is_close(truth_samples[-1][0], truth):
            truth_samples[-1][1].extend(samples)  # Group with previous truth
        else:
            truth_samples.append((truth, samples))

    # Filter those truths without enough samples
    truth_samples = [entry for entry in truth_samples if len(entry[1]) > 10]

    # Filter NaN
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
        pickle.dump(truth_samples, argv.output)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('predefined_truths', type=argparse.FileType('rb'))
    parser.add_argument('pickle_file', nargs='+', type=argparse.FileType('rb'))
    parser.add_argument('-o', '--output', type=argparse.FileType('wb'), help="Save output as a pickle file")
    main(parser.parse_args())
