#ifndef LANE_CENTERING_H
#define LANE_CENTERING_H

#include "../include/approx_real.h"

#define WHEEL_BASE ((ApproxReal) 1.75)
#define WHEEL_BASE_INV ((ApproxReal) 4.0/7)
#define LOOK_AHEAD (ONE * 6)
#define LOOK_AHEAD_INV (ONE / 6)
#define FORWARD_VEL ((ApproxReal) 2.8)
#define CYCLE_SEC (ONE / 20)
#define RATE_HZ (20 * ONE)
#define STEERING_MIN ((ApproxReal) -0.61)
#define STEERING_MAX ((ApproxReal) 0.61)

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

  ApproxReal alpha = perc.phi + approx_asin(approx_mul(perc.delta, LOOK_AHEAD_INV));
  ApproxReal error = approx_atan(
    approx_mul(approx_mul(2 * WHEEL_BASE, approx_sin(alpha)), LOOK_AHEAD_INV)
  );

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
  next_state.x = curr_state.x + approx_mul(approx_mul(FORWARD_VEL, CYCLE_SEC), approx_cos(curr_state.yaw));
  next_state.y = curr_state.y + approx_mul(approx_mul(FORWARD_VEL, CYCLE_SEC), approx_sin(curr_state.yaw));
  next_state.yaw = curr_state.yaw + approx_mul(approx_mul(approx_mul(FORWARD_VEL, CYCLE_SEC), WHEEL_BASE_INV),
                                               approx_tan(steering));
  return next_state;
}

#endif // LANE_CENTERING_H
