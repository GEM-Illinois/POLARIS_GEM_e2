#ifndef LANE_CENTERING_H
#define LANE_CENTERING_H

#include "../include/approx_real.h"

#define FORWARD_SPEED ((ApproxReal) 2.8)
// WHEEL_BASE = 1.75
#define WHEEL_BASE_INV ((ApproxReal) 4.0/7)
#define K_P ((ApproxReal) 0.45)
#define STEERING_MIN ((ApproxReal) -0.61)
#define STEERING_MAX ((ApproxReal) 0.61)
#define CYCLE_SEC (ONE / 20)
#define RATE_HZ (20 * ONE)

typedef struct _State {
    ApproxReal x;
    ApproxReal y;
    ApproxReal yaw;
} State;

typedef struct _Perception {
    ApproxReal phi;
    ApproxReal delta;
} Perception;

static inline Perception sensor(State curr_state) {
  // NOTE assume the mid-lane is aligned with x-axis (y==0 and yaw==0)
  Perception ret = {
    .phi = -curr_state.yaw,
    .delta = -curr_state.y
  };
  return ret;
}

static inline ApproxReal controller(Perception perc) {
  ApproxReal error = perc.phi + approx_atan2(approx_mul(K_P, perc.delta), FORWARD_SPEED);

  ApproxReal steering;
  if(error > STEERING_MAX) {
    steering = STEERING_MAX;
  }
  else if(error < STEERING_MIN) {
    steering = STEERING_MIN;
  }
  else {
    steering = error;
  }
  return steering;
}

static inline State dynamics(State curr_state, ApproxReal steering) {
  State next_state;
  next_state.x = curr_state.x + approx_mul(approx_mul(FORWARD_SPEED, CYCLE_SEC), approx_cos(curr_state.yaw + steering));
  next_state.y = curr_state.y + approx_mul(approx_mul(FORWARD_SPEED, CYCLE_SEC), approx_sin(curr_state.yaw + steering));
  next_state.yaw = curr_state.yaw + approx_mul(approx_mul(approx_mul(FORWARD_SPEED, CYCLE_SEC), WHEEL_BASE_INV),
                                               approx_sin(steering));
  return next_state;
}

#endif // LANE_CENTERING_H
