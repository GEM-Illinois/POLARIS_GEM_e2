#!/usr/bin/env python3

"""
Generate a list of desired ground truth perception output and stored as a yaml file.
"""

from typing import List, Tuple
import yaml

import numpy as np
from scipy.stats.distributions import truncnorm


def gen_truncated_normal_truths(
        psi_range: Tuple[float, float] ,
        cte_range: Tuple[float, float],
        num_truths: int) -> List[List[float]]:
    psi_min, psi_max = psi_range
    cte_min, cte_max = cte_range
    my_mean, my_std = np.zeros(2), np.array([0.05, 0.4])
    myclip_a, myclip_b = np.array([psi_min, cte_min]), np.array([psi_max, cte_max])
    a, b = (myclip_a - my_mean) / my_std, (myclip_b - my_mean) / my_std
    return truncnorm.rvs(a, b, size=(num_truths, 2), loc=my_mean, scale=my_std).tolist()


def gen_uniform_truths(
        psi_range: Tuple[float, float] ,
        cte_range: Tuple[float, float],
        num_truths: int) -> List[List[float]]:
    psi_min, psi_max = psi_range
    cte_min, cte_max = cte_range
    return np.random.uniform(np.array([psi_min, cte_min]), np.array([psi_max, cte_max]), (num_truths, 2)).tolist()


def gen_evenly_spaced_truths(
        psi_range: Tuple[float, float],
        cte_range: Tuple[float, float],
        num_psi: int, num_cte: int) -> List[List[float]]:
    return [[float(psi), float(cte)] for psi in np.linspace(psi_range[0], psi_range[1], num_psi)
            for cte in np.linspace(cte_range[0], cte_range[1], num_cte)]


def main():
    num_truths = 800  # type: int
    distribution = "partitioned_uniform"
    pi_div = 12  # type: int
    psi_max, cte_max = np.pi/pi_div, 1.2
    psi_min, cte_min = -psi_max, -cte_max

    if distribution == "truncated_normal":
        num_psi_parts, num_cte_parts = None, None
        truth_list = gen_truncated_normal_truths(psi_range=(psi_min, psi_max),
                                                 cte_range=(cte_min, cte_max),
                                                 num_truths=num_truths)
    elif distribution == "evenly_spaced":
        num_psi_parts, num_cte_parts = 11, 11
        assert num_psi_parts*num_cte_parts == num_truths
        truth_list = gen_evenly_spaced_truths(psi_range=(psi_min, psi_max),
                                              cte_range=(cte_min, cte_max),
                                              num_psi=num_psi_parts, num_cte=num_cte_parts)
    elif distribution == "uniform":
        num_psi_parts, num_cte_parts = None, None
        truth_list = gen_uniform_truths(psi_range=(psi_min, psi_max),
                                        cte_range=(cte_min, cte_max),
                                        num_truths=num_truths)
    elif distribution == "partitioned_uniform":
        num_psi_parts, num_cte_parts = 20, 4
        assert num_truths % (num_psi_parts*num_cte_parts) == 0
        num_truths_per_part = num_truths // (num_psi_parts*num_cte_parts)

        psi_arr = np.linspace(psi_min, psi_max, num=num_psi_parts + 1)
        list_psi_range = list(zip(psi_arr[0:num_psi_parts], psi_arr[1:num_psi_parts + 1]))

        cte_arr = np.linspace(cte_min, cte_max, num=num_cte_parts + 1)
        list_cte_range = list(zip(cte_arr[0:num_cte_parts], cte_arr[1:num_cte_parts + 1]))

        truth_list = []
        for psi_range in list_psi_range:
            for cte_range in list_cte_range:
                truth_list.extend(gen_uniform_truths(psi_range=psi_range,
                                                     cte_range=cte_range,
                                                     num_truths=num_truths_per_part))
    else:
        raise NotImplementedError("Unsupported probability distribution %s" % distribution)

    # NOTE Swap the order from ["psi", "cte"] to ["cte", "psi"]
    truth_list = [[v2, v1] for v1, v2 in truth_list]

    assert (num_psi_parts is None) == (num_cte_parts is None)

    if num_psi_parts is None and num_cte_parts is None:
        distr_str = str(distribution)
    else:
        distr_str = f"{distribution}_{num_cte_parts}x{num_psi_parts}"
    output_file_name = f"{num_truths}_truths-{distr_str}-{cte_max}m-pi_{pi_div}.yaml"

    print("Saving generated ground truths to file '%s'" % output_file_name)
    with open(output_file_name, "w") as out_file:
        yaml.safe_dump({
            "fields": {"truth": ["cte", "psi"]},
            "distribution": {
                str(distribution): {
                    "lb": [cte_min, psi_min],
                    "ub": [cte_max, psi_max],
                }
            },
            "truth_list": truth_list}, out_file, default_flow_style=None)


if __name__ == "__main__":
    main()

