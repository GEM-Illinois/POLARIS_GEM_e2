from numpy import arctan2, sin, cos

WHEEL_BASE = 1.75  # m

K_P = 0.45
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
    """ Stanley controller """
    prcv_heading, prcv_distance = epsilon
    error = prcv_heading + arctan2(K_P * prcv_distance, SPEED)

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
        *the front axle* as the vehicle frame.

        dx/dt = FORWARD_VEL*cos(yaw+steering)
        dy/dt = FORWARD_VEL*sin(yaw+steering)
        dyaw/dt = FORWARD_VEL*sin(steering)/WHEEL_BASE
    """
    old_x, old_y, old_yaw = old_state
    new_x = old_x + SPEED * cos(old_yaw + steering) * CYCLE_SEC
    new_y = old_y + SPEED * sin(old_yaw + steering) * CYCLE_SEC
    new_yaw = old_yaw + SPEED * sin(steering) / WHEEL_BASE * CYCLE_SEC
    return new_x, new_y, new_yaw


def main():
    state0 = (0.0, 0.0, 0.0)  # Can be any initial value

    epsilon = sensor(state0)
    steering = controller(epsilon)  # Note that controller is using perceived value to compute
    state1 = dynamics(state0, steering)
