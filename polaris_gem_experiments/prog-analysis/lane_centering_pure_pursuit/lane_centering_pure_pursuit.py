from numpy import arcsin, arctan, sin, cos, tan

WHEEL_BASE = 1.75  # meters
LOOK_AHEAD = 6.0  # m
SPEED = 2.8  # m/s
CYCLE_SEC = 0.05  # s
STEER_LIM = 0.61  # rad


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

    alpha = prcv_heading + arcsin(prcv_distance / LOOK_AHEAD)

    error = arctan(2 * WHEEL_BASE * sin(alpha) / LOOK_AHEAD)

    # Calculate controller output
    if error > STEER_LIM:
        steering = STEER_LIM
    elif error < -STEER_LIM:
        steering = -STEER_LIM
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
    new_x = old_x + SPEED * cos(old_yaw) * CYCLE_SEC
    new_y = old_y + SPEED * sin(old_yaw) * CYCLE_SEC
    new_yaw = old_yaw + SPEED * tan(steering) / WHEEL_BASE * CYCLE_SEC
    return new_x, new_y, new_yaw


def main():
    state0 = (0.0, 0.0, 0.0)  # Can be any initial value

    epsilon = sensor(state0)
    steering = controller(epsilon)  # Note that controller is using perceived value to compute
    state1 = dynamics(state0, steering)
