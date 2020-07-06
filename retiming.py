# (0, b'r_shoulder_out_joint', 0, 7, 6, 1, 0.0, 0.0, -2.07079632679, 1.57079632679, 1000.0, 2000.0, b'r_bicep_link_aux_0', (1.0, 0.0, 0.0), (0.0, -0.066, -0.06), (0.0, 0.0, 0.0, 1.0), -1)
# (1, b'r_shoulder_lift_joint', 0, 8, 7, 1, 0.0, 0.0, -2.07079632679, 1.57079632679, 1000.0, 2000.0, b'r_bicep_link_aux_1', (0.0, 1.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0)
# (2, b'r_upper_arm_roll_joint', 0, 9, 8, 1, 0.0, 0.0, -1.57079632679, 1.57079632679, 1000.0, 2000.0, b'r_bicep_link', (0.0, 0.0, 1.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 1)
# (3, b'r_elbow_flex_joint', 0, 10, 9, 1, 0.0, 0.0, -3.14159265359, 0.0, 1000.0, 2000.0, b'r_forearm_link_aux', (0.0, 1.0, 0.0), (-0.054, -0.009, -0.2621), (0.0, 0.0, 0.0, 1.0), 2)
# (4, b'r_elbow_roll_joint', 0, 11, 10, 1, 0.0, 0.0, -1.57079632679, 1.57079632679, 1000.0, 2000.0, b'r_forearm_link', (0.0, 0.0, 1.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 3)
# (5, b'r_wrist_roll_joint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'r_hand_link', (0.0, 0.0, 0.0), (0.0303, -0.00729, -0.302), (1.0, 0.0, 0.0, 1.0341155355510722e-13), 4)
# (6, b'rh_WRJ2', 0, 12, 11, 1, 0.1, 0.0, -1.0471975512, 1.0471975512, 10.0, 20.0, b'rh_wrist', (0.0, 1.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 5)
# (7, b'rh_WRJ1', 0, 13, 12, 1, 0.1, 0.0, -1.57079632679, 1.57079632679, 30.0, 20.0, b'rh_palm', (1.0, 0.0, 0.0), (0.0, 0.0, 0.0009999999999999974), (0.0, 0.0, 0.0, 1.0), 6)
import inspect
import os

import numpy as np
import pybullet as p

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

mode = p.DIRECT  # or p.DIRECT without GUI
p.connect(mode)
p.resetSimulation()
p.setTimeStep(1. / 240)
arm_dofs = [0, 1, 2, 3, 4, 6, 7]
maxForce = 200.
arm_id = p.loadURDF(os.path.join(currentdir, 'inmoov_arm_v2_2_reaching_BB.urdf'), [-0.30, 0.348, 0.272],
                    p.getQuaternionFromEuler([0, 0, 0]), flags=1, useFixedBase=1)


def dist_func(a, b, c, t):
    return a * t ** 3 + b * t ** 2 + c * t


def retimed_traj(Traj, v0=0, v1=0):
    """
    :param Traj: Original trajectory computed by OpenRAVE of shape (T, 7)
    :param v0: Magnitude of initial velocity
    :param v1: Magnitude of final velocity
    :return: Retimed trajectory of shape (T, 7). The magnitude of velocity satisfies quadratic functions.
    """
    # d(t) = at^3 + bt^2 + ct
    # d(0) = 0, d(1) = a + b + c = dist, d'(0) = c = v0, d'(1) = 3a + 2b + c = v1
    # a = v0 + v1 - 2dist, b = 3dist - 2v0 - v1, c = v0
    dist_list = [0]  # distance at each timestep
    tar_armq = Traj[0, 0:7]  # TODO: initial Q?
    p.setJointMotorControlArray(
        bodyIndex=arm_id,
        jointIndices=arm_dofs,
        controlMode=p.POSITION_CONTROL,
        targetPositions=list(tar_armq))  # TODO: no max force, guarantee for target q?
    p.stepSimulation()
    hand_pos, hand_quat, *_ = p.getLinkState(arm_id, 7, computeForwardKinematics=1)
    for ind in range(1, len(Traj)):
        tar_armq = Traj[ind, 0:7]
        p.setJointMotorControlArray(
            bodyIndex=arm_id,
            jointIndices=arm_dofs,
            controlMode=p.POSITION_CONTROL,
            targetPositions=list(tar_armq),
            forces=[maxForce * 3] * len(arm_dofs))
        p.stepSimulation()
        hand_pos_prime, hand_quat_prime, *_ = p.getLinkState(arm_id, 7, computeForwardKinematics=1)
        dist_list.append(np.linalg.norm(np.array(hand_pos_prime) - np.array(hand_pos)))
        hand_pos = hand_pos_prime
    dist = np.cumsum(np.array(dist_list))
    a = v0 + v1 - 2 * dist[-1]
    b = 3 * dist[-1] - 2 * v0 - v1
    c = v0
    T = np.arange(len(Traj)) / len(Traj)  # TODO: t_start = 0, t_end = 1?
    retimed_traj = np.zeros((Traj.shape[0], 7))
    retimed_traj[0] = Traj[0, 0:7]
    for t in range(1, len(Traj)):
        ts = t / len(Traj)
        dist_targ = dist_func(a, b, c, ts)
        idx = np.searchsorted(dist, dist_targ)
        dist0 = dist[idx - 1]
        dist1 = dist[idx]
        ppt = (dist_targ - dist0) / (dist1 - dist0)
        retimed_traj[t] = Traj[idx - 1, 0:7] + ppt * (Traj[idx, 0:7] - Traj[idx - 1, 0:7])
    return retimed_traj
