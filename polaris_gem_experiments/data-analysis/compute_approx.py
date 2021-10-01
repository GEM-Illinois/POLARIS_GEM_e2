#!/usr/bin/env python3

from itertools import islice
import pickle
from typing import Any

import numpy as np
from matplotlib import transforms as transforms, pyplot as plt
from matplotlib.patches import Ellipse, Rectangle

from sklearn.linear_model import LinearRegression


def confidence_ellipse(x, y, ax, n_std=3.0, facecolor='none', **kwargs):
    """
    Create a plot of the covariance confidence ellipse of *x* and *y*.
    See https://matplotlib.org/gallery/statistics/confidence_ellipse.html

    Parameters
    ----------
    x, y : array-like, shape (n, )
        Input data.

    ax : matplotlib.axes.Axes
        The axes object to draw the ellipse into.

    n_std : float
        The number of standard deviations to determine the ellipse's radiuses.

    **kwargs
        Forwarded to `~matplotlib.patches.Ellipse`

    Returns
    -------
    matplotlib.patches.Ellipse
    """
    if x.size != y.size:
        raise ValueError("x and y must be the same size")

    cov = np.cov(x, y)
    pearson = cov[0, 1]/np.sqrt(cov[0, 0] * cov[1, 1])
    # Using a special case to obtain the eigenvalues of this
    # two-dimensional dataset.
    ell_radius_x = np.sqrt(1 + pearson)
    ell_radius_y = np.sqrt(1 - pearson)
    ellipse = Ellipse((0, 0), width=ell_radius_x * 2, height=ell_radius_y * 2,
                      facecolor=facecolor, **kwargs)

    # Calculating the standard deviation of x from
    # the square-root of the variance and multiplying
    # with the given number of standard deviations.
    scale_x = np.sqrt(cov[0, 0]) * n_std
    mean_x = np.mean(x)

    # calculating the standard deviation of y ...
    scale_y = np.sqrt(cov[1, 1]) * n_std
    mean_y = np.mean(y)

    transf = transforms.Affine2D() \
        .rotate_deg(45) \
        .scale(scale_x, scale_y) \
        .translate(mean_x, mean_y)

    ellipse.set_transform(transf + ax.transData)
    return ax.add_patch(ellipse)


def confidence_ellipse_gaussian(mean: np.ndarray, cov: np.ndarray, ax, n_std=3.0, facecolor='none', **kwargs):
    mean_x, mean_y = mean
    pearson = cov[0, 1] / np.sqrt(cov[0, 0] * cov[1, 1])
    ell_radius_x = np.sqrt(1 + pearson)
    ell_radius_y = np.sqrt(1 - pearson)
    ellipse = Ellipse((0, 0), width=ell_radius_x * 2, height=ell_radius_y * 2,
                      facecolor=facecolor, **kwargs)
    scale_x = np.sqrt(cov[0, 0]) * n_std
    scale_y = np.sqrt(cov[1, 1]) * n_std

    rect = Rectangle((mean_x-scale_x, mean_y-scale_y), scale_x*2, scale_y*2,
                     edgecolor='r', facecolor='none', linestyle=':')
    ax.add_patch(rect)

    transf = transforms.Affine2D() \
        .rotate_deg(45) \
        .scale(scale_x, scale_y) \
        .translate(mean_x, mean_y)
    ellipse.set_transform(transf + ax.transData)
    return ax.add_patch(ellipse)


def plot_samples(truth_samples_seq, regressor, fit_cov, nrows=5, ncols=5):
    fig, axs = plt.subplots(nrows, ncols, sharex=True, sharey=True)
    fig.suptitle("Red dot is ground truth.\n"
                 "Yellow dot and ellipse is observed mean and 3*cov by the output of NN.\n"
                 # "Purple dot and ellipse is mean and estimated 3*cov of the fitted Gaussian"
                 # " by Linear Regression.\n"
                 "x is heading angle (radian). "
                 "y is distance to lane center (meter).")
    for idx, (truth, est_list) in enumerate(truth_samples_seq):
        print("Truth: (%.2f, %.2f);" % truth, "#Samples: %d" % len(est_list))
        i, j = divmod(idx, nrows)
        ax = axs[i][j]
        ax.set_xlim(-1.0, 1.0)
        ax.set_ylim(-3.0, 3.0)
        ax.plot(truth[0], truth[1], 'ro')

        # Plot samples
        obs_y_arr = np.array(est_list)
        ax.scatter(obs_y_arr[:, 0], obs_y_arr[:, 1], s=1.0)
        obs_mean = np.mean(obs_y_arr, axis=0)
        ax.plot(obs_mean[0], obs_mean[1], 'yo')
        confidence_ellipse(obs_y_arr[:, 0], obs_y_arr[:, 1], ax, edgecolor='y')

        # Plot regression model
        x_arr = [truth]
        fit_y = regressor.predict(x_arr)[0]
        ax.plot(fit_y[0], fit_y[1], 'mo')
        confidence_ellipse_gaussian(fit_y, fit_cov, ax, edgecolor='m')

    plt.show()


def key_for_subplots(entry):
    """ Generate keys to order the subplots so that, for the ground truth that is further from (0, 0),
        its plot is placed further from the center plot.
    """
    (heading, dist), _ = entry
    return -dist, heading


def est_error_cov(obs_y: np.ndarray, fit_y: np.ndarray, x_dim: int = 1):
    """ Estimate of error covariance using the mean SSCP error """
    assert obs_y.shape == fit_y.shape

    # Sum of squares and cross-products
    error_arr = obs_y - fit_y
    error_sscp = np.matmul(np.transpose(error_arr), error_arr)
    return error_sscp / (len(obs_y) - x_dim - 1)


def main(argv: Any) -> None:
    for pickle_file_io in argv.pickle_file:
        truth_samples_seq = pickle.load(pickle_file_io)

        # preprocessing
        pred = lambda t: np.pi/36 <= abs(t[0]) <= np.pi/18
        # pred = lambda t: 0.0 <= abs(t[0]) <= np.pi / 36
        # FIXME Imputation instead of removing sampled output?
        truth_samples_seq = [(t, [s for s in raw_samples if not any(np.isnan(s))])
                             for t, raw_samples in truth_samples_seq if pred(t)]

        x_list, y_list = [], []
        for truth, samples in truth_samples_seq:
            x_list.extend([truth]*len(samples))
            y_list.extend(samples)
        assert len(x_list) == len(y_list)
        x_arr = np.array(x_list)
        y_arr = np.array(y_list)
        regressor = LinearRegression(fit_intercept=False, copy_X=False)
        regressor.fit(x_arr, y_arr)

        if argv.output is not None:
            print("Save regression model to %s" % argv.output.name)
            pickle.dump(regressor, argv.output)

        fit_y_arr = regressor.predict(x_arr)
        cov = est_error_cov(y_arr, fit_y_arr, x_arr.shape[1])

        print("Coefficients: %s" % regressor.coef_.tolist())
        print("Intercept: %s" % regressor.intercept_)
        print("Estimated Error Covariance: %s" % cov.tolist())
        print("3*Standard Deviation: %s" % (3.0*np.sqrt([cov[0, 0], cov[1, 1]])).tolist())

        # Select some ground truths for pretty plots
        if argv.plot:
            nrows, ncols = 4, 4
            truth_samples_seq = islice(truth_samples_seq, 0, 0 + nrows * ncols)
            truth_samples_seq = list(truth_samples_seq)
            truth_samples_seq.sort(key=key_for_subplots)
            plot_samples(truth_samples_seq, regressor, cov, nrows, ncols)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('pickle_file', nargs='+', type=argparse.FileType('rb'))
    parser.add_argument('-p', '--plot', action='store_true', help="plot samples of each ground truth")
    parser.add_argument('-o', '--output', type=argparse.FileType('wb'), help="Save model to output as pickle file")
    main(parser.parse_args())
