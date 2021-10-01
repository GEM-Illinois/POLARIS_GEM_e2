#!/usr/bin/env python3

"""
Generate a list of desired ground truth perception output and stored as a yaml file.
"""

from typing import List, Tuple
import yaml

import numpy as np
from scipy.stats.distributions import truncnorm


def gen_truncated_normal_truths(
        phi_range: Tuple[float, float] ,
        cte_range: Tuple[float, float],
        num_truths: int) -> List[List[float]]:
    phi_min, phi_max = phi_range
    cte_min, cte_max = cte_range
    my_mean, my_std = np.zeros(2), 0.1*np.ones(2)
    myclip_a, myclip_b = np.array([phi_min, cte_min]), np.array([phi_max, cte_max])
    a, b = (myclip_a - my_mean) / my_std, (myclip_b - my_mean) / my_std
    return truncnorm.rvs(a, b, size=(num_truths, 2), loc=my_mean, scale=my_std).tolist()


def gen_uniform_truths(
        phi_range: Tuple[float, float] ,
        cte_range: Tuple[float, float],
        num_truths: int) -> List[List[float]]:
    phi_min, phi_max = phi_range
    cte_min, cte_max = cte_range
    return np.random.uniform(np.array([phi_min, cte_min]), np.array([phi_max, cte_max]), (num_truths, 2)).tolist()


def gen_evenly_spaced_truths(
        phi_range: Tuple[float, float],
        cte_range: Tuple[float, float],
        num_phi: int, num_cte: int) -> List[List[float]]:
    return [[phi, cte] for phi in np.linspace(phi_range[0], phi_range[1], num_phi)
            for cte in np.linspace(cte_range[0], cte_range[1], num_cte)]


def main():
    phi_min, phi_max = -np.pi/6, np.pi/6
    cte_min, cte_max = -1.2, 1.2
    num_truths = 100
    distribution = "uniform"
    output_file_name = "truths-%s-pi_6-1.2m-%d.yaml" % (distribution, num_truths)

    if distribution == "truncated_normal":
        truth_list = gen_truncated_normal_truths(phi_range=(phi_min, phi_max),
                                                 cte_range=(cte_min, cte_max),
                                                 num_truths=num_truths)
    elif distribution == "evenly_spaced":
        truth_list = gen_evenly_spaced_truths(phi_range=(phi_min, phi_max),
                                              cte_range=(cte_min, cte_max),
                                              num_phi=6, num_cte=6)
    elif distribution == "uniform":
        truth_list = gen_uniform_truths(phi_range=(phi_min, phi_max),
                                        cte_range=(cte_min, cte_max),
                                        num_truths=num_truths)
    else:
        raise NotImplementedError("Unsupported probability distribution %s" % distribution)

    print("Saving generated ground truths to file '%s'" % output_file_name)
    with open(output_file_name, "w") as out_file:
        yaml.safe_dump({"truth_list": truth_list}, out_file, default_flow_style=None)


if __name__ == "__main__":
    main()