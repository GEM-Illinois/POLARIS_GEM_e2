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
    return [[float(phi), float(cte)] for phi in np.linspace(phi_range[0], phi_range[1], num_phi)
            for cte in np.linspace(cte_range[0], cte_range[1], num_cte)]


def main():
    num_truths = 121  # type: int
    distribution = "evenly_spaced"
    phi_div = 12  # type: int
    phi_max, cte_max = np.pi/phi_div, 1.2
    phi_min, cte_min = -phi_max, -cte_max

    if distribution == "truncated_normal":
        truth_list = gen_truncated_normal_truths(phi_range=(phi_min, phi_max),
                                                 cte_range=(cte_min, cte_max),
                                                 num_truths=num_truths)
        output_file_name = "%d_truths-%s-pi_%d-%.1fm.yaml" % (num_truths, distribution, phi_div, cte_max)
    elif distribution == "evenly_spaced":
        num_phi, num_cte = 11, 11
        assert num_phi*num_cte == num_truths
        truth_list = gen_evenly_spaced_truths(phi_range=(phi_min, phi_max),
                                              cte_range=(cte_min, cte_max),
                                              num_phi=num_phi, num_cte=num_cte)
        output_file_name = "%d_truths-%s_%dx%d-pi_%d-%.1fm.yaml" \
                           % (num_truths, distribution, num_phi, num_cte, phi_div, cte_max)
    elif distribution == "uniform":
        truth_list = gen_uniform_truths(phi_range=(phi_min, phi_max),
                                        cte_range=(cte_min, cte_max),
                                        num_truths=num_truths)
        output_file_name = "%d_truths-%s-pi_%d-%.1fm.yaml" % (num_truths, distribution, phi_div, cte_max)
    else:
        raise NotImplementedError("Unsupported probability distribution %s" % distribution)

    print("Saving generated ground truths to file '%s'" % output_file_name)
    with open(output_file_name, "w") as out_file:
        yaml.safe_dump({"truth_list": truth_list}, out_file, default_flow_style=None)


if __name__ == "__main__":
    main()

