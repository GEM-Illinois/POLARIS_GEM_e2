#include <assert.h>

#include "lane_centering_pure_pursuit.h"

#define HALF_LANE_W ((ApproxReal) 2.0)

#define PHI_MAX APPROX_PI/12
#define CTE_MAX ((ApproxReal) 1.2)

static inline State nondet_initial_state() {
  State init_state = {
      .x=nondet_approx_real(),
      .y=nondet_approx_real(),
      .yaw=nondet_approx_real()
  };
  // Contraints on the state
  __VERIFIER_assume(-10*ONE <= init_state.x && init_state.x <= 10*ONE);
  __VERIFIER_assume(-CTE_MAX <= init_state.y && init_state.y <= CTE_MAX);
  __VERIFIER_assume(-PHI_MAX <= init_state.yaw && init_state.yaw <= PHI_MAX);

  // __VERIFIER_assume(0 <= init_state.y && 0 <= init_state.yaw);
  return init_state;
}

static inline ApproxReal lyapunov(Perception perc)
{
  ApproxReal alpha = perc.phi + approx_asin(approx_mul(perc.delta, LOOK_AHEAD_INV));
  ApproxReal error = approx_atan(
      approx_mul(approx_mul(2 * WHEEL_BASE, approx_sin(alpha)), LOOK_AHEAD_INV)
  );
  return approx_abs(error);
}

Perception nondet_approx(Perception truth) {
  Perception ret = {
    .phi = nondet_approx_real(),
    .delta = nondet_approx_real()
  };
  // Constraints to avoid overflow
  __VERIFIER_assume(-APPROX_PI_2 <= ret.phi && ret.phi <= APPROX_PI_2);
  __VERIFIER_assume(-HALF_LANE_W <= ret.delta && ret.delta <= HALF_LANE_W);

  Perception est = {
    .phi = ((ApproxReal)1.0439444665079134)*truth.phi + ((ApproxReal)0.02020563578563709)*truth.delta,
    .delta = ((ApproxReal)-0.7665235072555745)*truth.phi + ((ApproxReal)0.7699710804750737)*truth.delta
  };

  ApproxReal dist = approx_abs(ret.phi - est.phi) + approx_abs(ret.delta - est.delta);  // L1-norm
  assert(dist >= 0);

  #define PHI_STEP PHI_MAX/10
  #define CTE_STEP CTE_MAX/2
  if(-2*CTE_STEP <= truth.delta && truth.delta <= -1*CTE_STEP)
  {
    if(0 <= truth.phi && truth.phi <= PHI_STEP)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.067));}
    else if(PHI_STEP*1 <= truth.phi && truth.phi <= PHI_STEP*2)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.049));}
    else if(PHI_STEP*2 <= truth.phi && truth.phi <= PHI_STEP*3)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.030));}
    else if(PHI_STEP*3 <= truth.phi && truth.phi <= PHI_STEP*4)
    { __VERIFIER_assume(0);/* skip the branch */}
    else if(PHI_STEP*4 <= truth.phi && truth.phi <= PHI_STEP*5)
    { __VERIFIER_assume(0);/* skip the branch */}
    else if(PHI_STEP*5 <= truth.phi && truth.phi <= PHI_STEP*6)
    { __VERIFIER_assume(0);/* skip the branch */}
    else if(PHI_STEP*6 <= truth.phi && truth.phi <= PHI_STEP*7)
    { __VERIFIER_assume(0);/* skip the branch */}
    else if(PHI_STEP*7 <= truth.phi && truth.phi <= PHI_STEP*8)
    { __VERIFIER_assume(0);/* skip the branch */}
    else if(PHI_STEP*8 <= truth.phi && truth.phi <= PHI_STEP*9)
    { __VERIFIER_assume(0);/* skip the branch */}
    else if(PHI_STEP*9 <= truth.phi)
    { __VERIFIER_assume(0);/* skip the branch */}
    else
    { __VERIFIER_assume(0);/* skip the branch */}
  }
  else if(-1*CTE_STEP <= truth.delta && truth.delta <= 0)
  {
    if(0 <= truth.phi && truth.phi <= PHI_STEP)
    { __VERIFIER_assume(0);/* skip the branch */}
    else if(PHI_STEP*1 <= truth.phi && truth.phi <= PHI_STEP*2)
    { __VERIFIER_assume(0);/* skip the branch */}
    else if(PHI_STEP*2 <= truth.phi && truth.phi <= PHI_STEP*3)
    { __VERIFIER_assume(0);/* skip the branch */}
    else if(PHI_STEP*3 <= truth.phi && truth.phi <= PHI_STEP*4)
    { __VERIFIER_assume(0);/* skip the branch */}
    else if(PHI_STEP*4 <= truth.phi && truth.phi <= PHI_STEP*5)
    { __VERIFIER_assume(0);/* skip the branch */}
    else if(PHI_STEP*5 <= truth.phi && truth.phi <= PHI_STEP*6)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.005));}
    else if(PHI_STEP*6 <= truth.phi && truth.phi <= PHI_STEP*7)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.024));}
    else if(PHI_STEP*7 <= truth.phi && truth.phi <= PHI_STEP*8)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.042));}
    else if(PHI_STEP*8 <= truth.phi && truth.phi <= PHI_STEP*9)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.061));}
    else if(PHI_STEP*9 <= truth.phi)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.079));}
    else
    { __VERIFIER_assume(0);/* skip the branch */}
  }
  else if(0 <= truth.delta && truth.delta <= CTE_STEP)
  {
    if(0 <= truth.phi && truth.phi <= PHI_STEP)
    { __VERIFIER_assume(0);/* skip the branch */}
    else if(PHI_STEP*1 <= truth.phi && truth.phi <= PHI_STEP*2)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.018));}
    else if(PHI_STEP*2 <= truth.phi && truth.phi <= PHI_STEP*3)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.036));}
    else if(PHI_STEP*3 <= truth.phi && truth.phi <= PHI_STEP*4)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.055));}
    else if(PHI_STEP*4 <= truth.phi && truth.phi <= PHI_STEP*5)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.073));}
    else if(PHI_STEP*5 <= truth.phi && truth.phi <= PHI_STEP*6)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.091));}
    else if(PHI_STEP*6 <= truth.phi && truth.phi <= PHI_STEP*7)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.110));}
    else if(PHI_STEP*7 <= truth.phi && truth.phi <= PHI_STEP*8)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.128));}
    else if(PHI_STEP*8 <= truth.phi && truth.phi <= PHI_STEP*9)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.147));}
    else if(PHI_STEP*9 <= truth.phi)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.165));}
    else
    { __VERIFIER_assume(0);/* skip the branch */}
  }
  else if(CTE_STEP <= truth.delta && truth.delta <= 2*CTE_STEP)
  {
    if(PHI_STEP*0 <= truth.phi && truth.phi <= PHI_STEP*1)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.086));}
    else if(PHI_STEP*1 <= truth.phi && truth.phi <= PHI_STEP*2)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.104));}
    else if(PHI_STEP*2 <= truth.phi && truth.phi <= PHI_STEP*3)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.123));}
    else if(PHI_STEP*3 <= truth.phi && truth.phi <= PHI_STEP*4)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.141));}
    else if(PHI_STEP*4 <= truth.phi && truth.phi <= PHI_STEP*5)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.159));}
    else if(PHI_STEP*5 <= truth.phi && truth.phi <= PHI_STEP*6)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.178));}
    else if(PHI_STEP*6 <= truth.phi && truth.phi <= PHI_STEP*7)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.196));}
    else if(PHI_STEP*7 <= truth.phi && truth.phi <= PHI_STEP*8)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.215));}
    else if(PHI_STEP*8 <= truth.phi && truth.phi <= PHI_STEP*9)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.233));}
    else if(PHI_STEP*9 <= truth.phi)
    { __VERIFIER_assume(dist <= ONE*((ApproxReal)0.252));}
    else
    { __VERIFIER_assume(0);/* skip the branch */}
  }
  else
  { __VERIFIER_assume(0);/* skip the branch */}
  return ret;
}

int main() {
  State old_state = nondet_initial_state();

  Perception old_truth = sensor(old_state);

  // Perception old_perc = old_truth;
  Perception old_perc = nondet_approx(old_truth);

  ApproxReal steering = controller(old_truth);

  State new_state = dynamics(old_state, steering);

  Perception new_truth = sensor(new_state);
perc.phi + approx_atan2(approx_mul(K_P, perc.delta), FORWARD_SPEED)
#ifdef ASSERT_COUNTERACT
  assert(STEERING_MIN <= steering && steering <= STEERING_MAX);

  if (old_truth.delta == 0) {
    assert(!(old_truth.phi >= 0) || steering >= 0);
    assert(!(old_truth.phi <= 0) || steering <= 0);
  }
  if (old_perc.phi == 0) {
    assert(!(old_truth.delta >= 0) || steering >= 0);
    assert(!(old_truth.delta <= 0) || steering <= 0);
  }
#endif

#ifdef ASSERT_NONINCREASE
  assert(lyapunov(new_truth) <= lyapunov(old_truth));
#endif
  return 0;
}
