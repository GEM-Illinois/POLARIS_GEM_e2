import numpy as np
from numpy import arctan2, sin, cos

WHEEL_BASE = 1.75  # meters

LOOK_AHEAD = 6.0  # m
FORWARD_VEL = 2.8  # m/s
CYCLE_TIME = 0.05  # s
STEERING_MIN = -0.61  # rad/s^2
STEERING_MAX = 0.61  # rad/s^2


def sensor(state):
    """ Assuming the lane to track is aligned with x-axis (i.e., y==0 and yaw==0)
        Hence, heading = 0-yaw = -yaw and distance = 0-y = -y."""
    x, y, yaw = state
    # TODO approximation instead of perfect perception
    prcv_heading = -yaw
    prcv_distance = -y
    return prcv_heading, prcv_distance


def controller(epsilon):
    """ Pure pursuit controller

    We assume the path to track is the perceived lane center line defined by
    heading and distance w.r.t the rear axle of the vehicle.

    """
    prcv_heading, prcv_distance = epsilon

    alpha = prcv_heading + np.arcsin(prcv_distance / LOOK_AHEAD)

    error = np.arctan(2 * WHEEL_BASE * np.sin(alpha) / LOOK_AHEAD)

    # Calculate controller output
    if error > STEERING_MAX:
        steering = STEERING_MAX
    elif error < STEERING_MIN:
        steering = STEERING_MIN
    else:
        steering = error

    # Return actuator values
    return steering


def dynamics(old_state, steering):
    """ This is the Kinematic Bicycle Model for state variables x, y, yaw using
        *the rear axle* as the vehicle frame.

        dx/dt = FORWARD_VEL*cos(yaw)
        dy/dt = FORWARD_VEL*sin(yaw)
        dyaw/dt = FORWARD_VEL*tan(steering)/WHEEL_BASE
    """
    old_x, old_y, old_yaw = old_state
    new_x = old_x + FORWARD_VEL * cos(old_yaw) * CYCLE_TIME
    new_y = old_y + FORWARD_VEL * sin(old_yaw) * CYCLE_TIME
    new_yaw = old_yaw + FORWARD_VEL * tan(steering) / WHEEL_BASE * CYCLE_TIME
    return new_x, new_y, new_yaw


def main():
    state0 = (0.0, 0.0, 0.0)  # Can be any initial value

    epsilon = sensor(state0)
    steering = controller(epsilon)  # Note that controller is using perceived value to compute
    state1 = dynamics(state0, steering)
