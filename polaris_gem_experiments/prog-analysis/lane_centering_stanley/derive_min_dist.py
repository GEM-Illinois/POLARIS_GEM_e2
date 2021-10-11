#!/usr/bin/env python3
import math
from typing import Tuple

import gurobipy as gp
from gurobipy import GRB

WHEEL_BASE = 1.75  # m

K_P = 0.45

CYCLE_TIME = 0.05  # second
FORWARD_VEL = 2.8  # m/s
STEERING_LIM = 0.61  # rad

# Limits on unconstrained variables to avoid overflow and angle normalization
ANG_LIM = math.pi / 2  # radian
CTE_LIM = 2.0  # meter

# Bounds for intermediate variables.
# These are not necessary but can improve efficiency
K_CTE_V_LIM = K_P * CTE_LIM / FORWARD_VEL
ATAN_K_CTE_V_LIM = math.atan(K_CTE_V_LIM)
RAW_ANG_ERR_LIM = (ANG_LIM + ATAN_K_CTE_V_LIM)

NEW_K_CTE_V_LIM = K_CTE_V_LIM + FORWARD_VEL * CYCLE_TIME * 1.0
NEW_ATAN_K_CTE_V_LIM = math.atan(NEW_K_CTE_V_LIM)
NEW_RAW_ANG_ERR_LIM = ANG_LIM + FORWARD_VEL * CYCLE_TIME


def add_constraints(m: gp.Model, y_bound: Tuple[float, float], yaw_bound: Tuple[float, float]) -> None:
    # Vehicle state variables
    old_x = m.addVar(name='x', vtype=GRB.CONTINUOUS, lb=0.0, ub=10.0)
    old_y = m.addVar(name='y', vtype=GRB.CONTINUOUS, lb=y_bound[0], ub=y_bound[1])
    old_yaw = m.addVar(name='θ', vtype=GRB.CONTINUOUS, lb=yaw_bound[0], ub=yaw_bound[1])
    # Perceived heading error and cross-track error
    phi = m.addVar(name='φ', vtype=GRB.CONTINUOUS, lb=-ANG_LIM, ub=ANG_LIM)
    cte = m.addVar(name='d', vtype=GRB.CONTINUOUS, lb=-CTE_LIM, ub=CTE_LIM)

    # Intermediate Variables
    K_cte_V = m.addVar(name="K*d/Vf", lb=-K_CTE_V_LIM, ub=K_CTE_V_LIM)
    m.addConstr(K_cte_V == K_P * cte / FORWARD_VEL)

    atan_K_cte_V = m.addVar(name="atan(K*d/Vf)", lb=-ATAN_K_CTE_V_LIM, ub=ATAN_K_CTE_V_LIM)
    m.addGenConstrTan(xvar=atan_K_cte_V, yvar=K_cte_V)

    # Clip steering angle
    error = m.addVar(name="(φ+atan(K*d/Vf))",
                     lb=-RAW_ANG_ERR_LIM, ub=RAW_ANG_ERR_LIM)
    m.addConstr(error == (phi + atan_K_cte_V))

    steering = m.addVar(name="δ", lb=-STEERING_LIM, ub=STEERING_LIM)
    m.addGenConstrPWL(name="clip", xvar=error, yvar=steering,
                      xpts=[-RAW_ANG_ERR_LIM, -STEERING_LIM, STEERING_LIM, RAW_ANG_ERR_LIM],
                      ypts=[-STEERING_LIM, -STEERING_LIM, STEERING_LIM, STEERING_LIM])

    yaw_steer = m.addVar(name="θ+δ",
                         lb=-(ANG_LIM + STEERING_LIM), ub=(ANG_LIM + STEERING_LIM))
    m.addConstr(yaw_steer == (old_yaw + steering))

    cos_yaw_steer = m.addVar(name="cos(θ+δ)", vtype=GRB.CONTINUOUS, lb=-1.0, ub=1.0)
    m.addGenConstrCos(yaw_steer, cos_yaw_steer)
    sin_yaw_steer = m.addVar(name="sin(θ+δ)", vtype=GRB.CONTINUOUS, lb=-1.0, ub=1.0)
    m.addGenConstrSin(yaw_steer, sin_yaw_steer)

    sin_steer = m.addVar(name="sinδ", vtype=GRB.CONTINUOUS, lb=-1.0, ub=1.0)
    m.addGenConstrSin(steering, sin_steer)

    new_x = old_x + FORWARD_VEL * CYCLE_TIME * cos_yaw_steer
    new_y = old_y + FORWARD_VEL * CYCLE_TIME * sin_yaw_steer
    new_yaw = old_yaw + sin_steer * FORWARD_VEL * CYCLE_TIME / WHEEL_BASE

    K_old_y_V = m.addVar(name="K*-y/Vf", vtype=GRB.CONTINUOUS, lb=-K_CTE_V_LIM, ub=K_CTE_V_LIM)
    m.addConstr(K_old_y_V == K_P * -old_y / FORWARD_VEL)
    atan_K_old_y_V = m.addVar(name="atan(K*-y/Vf)", vtype=GRB.CONTINUOUS, lb=-ATAN_K_CTE_V_LIM, ub=ATAN_K_CTE_V_LIM)
    m.addGenConstrTan(xvar=atan_K_old_y_V, yvar=K_old_y_V)

    K_new_y_V = m.addVar(name="K*-y'/Vf", vtype=GRB.CONTINUOUS, lb=-NEW_K_CTE_V_LIM, ub=NEW_K_CTE_V_LIM)
    m.addConstr(K_new_y_V == K_P * -new_y / FORWARD_VEL)
    atan_K_new_y_V = m.addVar(name="atan(K*-y'/Vf)", vtype=GRB.CONTINUOUS,
                              lb=-NEW_ATAN_K_CTE_V_LIM, ub=NEW_ATAN_K_CTE_V_LIM)
    m.addGenConstrTan(xvar=atan_K_new_y_V, yvar=K_new_y_V)

    old_err = m.addVar(name="-θ+atan(K*-y/Vf)", vtype=GRB.CONTINUOUS,
                       lb=-RAW_ANG_ERR_LIM, ub=RAW_ANG_ERR_LIM)
    m.addConstr(old_err == -old_yaw + atan_K_old_y_V)
    old_V = m.addVar(name="V(x,y,θ)", vtype=GRB.CONTINUOUS,
                     lb=0, ub=RAW_ANG_ERR_LIM)
    m.addConstr(old_V == gp.abs_(old_err))

    new_err = m.addVar(name="-θ'+atan(K*-y'/Vf)", vtype=GRB.CONTINUOUS,
                       lb=-NEW_RAW_ANG_ERR_LIM, ub=NEW_RAW_ANG_ERR_LIM)
    m.addConstr(new_err == -new_yaw + atan_K_new_y_V)
    new_V = m.addVar(name="V(x',y',θ')", vtype=GRB.CONTINUOUS,
                     lb=0, ub=NEW_RAW_ANG_ERR_LIM)
    m.addConstr(new_V == gp.abs_(new_err))

    m.addConstr(new_V >= old_V)  # Tracking error is increasing (UNSAFE)

    phi_diff = m.addVar(name="φ-(-θ)", vtype=GRB.CONTINUOUS, lb=-math.inf, ub=math.inf)
    m.addConstr(phi_diff == phi - (1.0439444665079134*(-old_yaw) + 0.02020563578563709*(-old_y)))
    phi_dist = m.addVar(name="|φ-(-θ)|", vtype=GRB.CONTINUOUS, lb=0, ub=math.inf)
    m.addConstr(phi_dist == gp.abs_(phi_diff))

    cte_diff = m.addVar(name="d-(-y)", vtype=GRB.CONTINUOUS, lb=-math.inf, ub=math.inf)
    m.addConstr(cte_diff == cte - (-0.7665235072555745*(-old_yaw) + 0.7699710804750737*(-old_y)))
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
                        # m.write("lane_centering_stanley.lp")

                        m.optimize()
                        y = m.getVarByName('y')
                        yaw = m.getVarByName('θ')
                        print('Obj: %g, ObjBound: %g, y: %g, yaw: %g\n'
                              % (m.objVal, m.objBound, y.x, yaw.x))

    except gp.GurobiError as e:
        print('Error code ' + str(e.errno) + ': ' + str(e))

    except AttributeError as e:
        print('Encountered an attribute error: ' + str(e))


if __name__ == "__main__":
    main()
