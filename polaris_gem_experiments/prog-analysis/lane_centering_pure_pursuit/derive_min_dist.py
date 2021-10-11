#!/usr/bin/env python3
import math
from typing import Tuple

import gurobipy as gp
from gurobipy import GRB

WHEEL_BASE = 1.75  # m
LOOK_AHEAD = 6.0  # m
FORWARD_VEL = 2.8  # m/s
CYCLE_TIME = 0.05  # second
STEERING_LIM = 0.61  # rad

# Limits on unconstrained variables to avoid overflow and angle normalization
ANG_LIM = math.pi / 2  # radian
CTE_LIM = 2.0  # meter
SAFE_CTE_LIM = 1.23

# Bounds for intermediate variables.
# These are not necessary but can improve efficiency
CTE_LOOK_AHEAD_LIM = CTE_LIM / LOOK_AHEAD
RAW_ANG_ERR_LIM = math.atan(2*WHEEL_BASE/LOOK_AHEAD)


def add_constraints(m: gp.Model, y_bound: Tuple[float, float], yaw_bound: Tuple[float, float]) -> None:
    # Vehicle state variables
    old_x = m.addVar(name='x', vtype=GRB.CONTINUOUS, lb=0.0, ub=10.0)
    old_y = m.addVar(name='y', vtype=GRB.CONTINUOUS, lb=y_bound[0], ub=y_bound[1])
    old_yaw = m.addVar(name='θ', vtype=GRB.CONTINUOUS, lb=yaw_bound[0], ub=yaw_bound[1])
    # Perceived heading error and cross-track error
    phi = m.addVar(name='φ', vtype=GRB.CONTINUOUS, lb=-ANG_LIM, ub=ANG_LIM)
    cte = m.addVar(name='d', vtype=GRB.CONTINUOUS, lb=-CTE_LIM, ub=CTE_LIM)

    # Intermediate Variables
    cte_ld = m.addVar(name="d/ld", lb=-CTE_LOOK_AHEAD_LIM, ub=CTE_LOOK_AHEAD_LIM)
    m.addConstr(cte_ld == cte / LOOK_AHEAD)

    asin_cte_ld = m.addVar(name="asin(d/ld)", lb=-ANG_LIM, ub=ANG_LIM)
    m.addGenConstrSin(xvar=asin_cte_ld, yvar=cte_ld)

    alpha = m.addVar(name="α", lb=-math.pi, ub=math.pi)
    m.addConstr(alpha == phi + asin_cte_ld)

    sin_alpha = m.addVar(name="sin(α)", lb=-1.0, ub=1.0)
    m.addGenConstrSin(xvar=alpha, yvar=sin_alpha)

    sin_alpha_2L_ld = m.addVar(name="2*L*sin(α)/ld", lb=-2*WHEEL_BASE/LOOK_AHEAD, ub=2*WHEEL_BASE/LOOK_AHEAD)
    m.addConstr(sin_alpha_2L_ld == 2*WHEEL_BASE*sin_alpha / LOOK_AHEAD)

    # Clip steering angle
    error = m.addVar(name="atan(2*L*sin(α)/ld)",
                     lb=-RAW_ANG_ERR_LIM, ub=RAW_ANG_ERR_LIM)
    m.addGenConstrTan(error, sin_alpha_2L_ld)

    steering = m.addVar(name="δ", lb=-STEERING_LIM, ub=STEERING_LIM)
    m.addGenConstrPWL(name="clip", xvar=error, yvar=steering,
                      xpts=[-ANG_LIM, -STEERING_LIM, STEERING_LIM, ANG_LIM],
                      ypts=[-STEERING_LIM, -STEERING_LIM, STEERING_LIM, STEERING_LIM])

    cos_yaw = m.addVar(name="cos(θ)", vtype=GRB.CONTINUOUS, lb=-1.0, ub=1.0)
    m.addGenConstrCos(old_yaw, cos_yaw)
    sin_yaw = m.addVar(name="sin(θ)", vtype=GRB.CONTINUOUS, lb=-1.0, ub=1.0)
    m.addGenConstrSin(old_yaw, sin_yaw)

    tan_steer = m.addVar(name="tan(δ)", vtype=GRB.CONTINUOUS, lb=-math.inf, ub=math.inf)
    m.addGenConstrTan(steering, tan_steer)

    new_x = old_x + FORWARD_VEL * CYCLE_TIME * cos_yaw

    new_y = m.addVar(name="y'", vtype=GRB.CONTINUOUS, lb=-CTE_LIM, ub=CTE_LIM)
    m.addConstr(new_y == old_y + FORWARD_VEL * CYCLE_TIME * sin_yaw)

    new_yaw = old_yaw + tan_steer * FORWARD_VEL * CYCLE_TIME / WHEEL_BASE

    m.addConstr(-SAFE_CTE_LIM <= old_y)
    m.addConstr(old_y <= SAFE_CTE_LIM)

    abs_new_y = m.addVar(name="|y'|", vtype=GRB.CONTINUOUS, lb=0, ub=math.inf)
    m.addConstr(abs_new_y == gp.abs_(new_y))
    m.addConstr(abs_new_y >= SAFE_CTE_LIM, name="¬Inv(x',y',θ')")

    phi_diff = m.addVar(name="φ-(-θ)", vtype=GRB.CONTINUOUS, lb=-math.inf, ub=math.inf)
    # m.addConstr(phi_diff == phi - (1.0439444665079134*(-old_yaw) + 0.02020563578563709*(-old_y)))
    m.addConstr(phi_diff == phi - (-old_yaw))
    phi_dist = m.addVar(name="|φ-(-θ)|", vtype=GRB.CONTINUOUS, lb=0, ub=math.inf)
    m.addConstr(phi_dist == gp.abs_(phi_diff))

    cte_diff = m.addVar(name="d-(-y)", vtype=GRB.CONTINUOUS, lb=-math.inf, ub=math.inf)
    # m.addConstr(cte_diff == cte - (-0.7665235072555745*(-old_yaw) + 0.7699710804750737*(-old_y)))
    m.addConstr(cte_diff == cte - (-old_y))
    cte_dist = m.addVar(name="|d-(-y)|", vtype=GRB.CONTINUOUS, lb=0, ub=math.inf)
    m.addConstr(cte_dist == gp.abs_(cte_diff))

    # L1-norm objective
    m.setObjective(phi_dist + cte_dist, GRB.MINIMIZE)
    # L2-norm objective
    # m.setObjective(phi_diff*phi_diff + cte_diff*cte_diff, GRB.MINIMIZE)
    return


def main():
    try:
        with gp.Env(empty=True) as env:
            env.setParam('OutputFlag', 0)
            env.start()
            STEP = 1
            # Bounds on initial states
            INIT_PHI_LIM = math.pi / 12
            INIT_CTE_LIM = 1.2
            for i in range(-2, 2, STEP):
                tmp_y_bound = (0.5 * i * INIT_CTE_LIM, 0.5 * (i + STEP) * INIT_CTE_LIM)
                y_bound = min(*tmp_y_bound), max(*tmp_y_bound)
                for j in range(-10, 10, STEP):
                    with gp.Model("lane_centering_stanley", env=env) as m:
                        tmp_yaw_bound = (0.1 * j * INIT_PHI_LIM, 0.1 * (j+STEP) * INIT_PHI_LIM)
                        yaw_bound = min(*tmp_yaw_bound), max(*tmp_yaw_bound)
                        print("Partition: y in [%g, %g] and yaw in [%g, %g]" % (y_bound + yaw_bound))
                        add_constraints(m, y_bound, yaw_bound)
                        m.write("lane_centering_pure_pursuit.lp")

                        m.optimize()
                        if m.status == GRB.OPTIMAL:
                            y = m.getVarByName('y')
                            yaw = m.getVarByName('θ')
                            print('Obj: %g, ObjBound: %g, y: %g, yaw: %g\n'
                                  % (m.objVal, m.objBound, y.x, yaw.x))
                        elif m.status == GRB.INF_OR_UNBD:
                            print('Model is infeasible or unbounded')
                        elif m.status == GRB.INFEASIBLE:
                            print('Model is infeasible')
                        elif m.status == GRB.UNBOUNDED:
                            print('Model is unbounded')
                        else:
                            print('Optimization ended with status %d' % m.status)
                        return

    except gp.GurobiError as e:
        print('Error code ' + str(e.errno) + ': ' + str(e))

    # except AttributeError as e:
    #    print('Encountered an attribute error: ' + str(e))


if __name__ == "__main__":
    main()
