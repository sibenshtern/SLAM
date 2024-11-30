import numpy as np
from scipy.spatial.transform import Rotation as R

import align


def compute_ate(timestamps_gt, positions_gt, quaternions_gt, timestamps_a, positions_a, quaternions_a):
    """
    compute absolute trajectory error (ATE) for positions and rotations

    Parameters
    ----------
    timestamps_gt, positions_gt, quaternions_gt : numpy.ndarray
        parameters from groundtruth
    timestamps_a, positions_a, quaternions_a : numpy.ndarray
        parameters from algorithm
    
    Returns
    -------
    ate_pos : float
        Root Mean Square Error (RMSE) for position ATE.
    ate_rot : float
        Mean rotational error in radians.
    """
    ate_pos = []
    ate_rot = []

    trajectories_gt_aligned, positions_gt_aligned, quaternions_gt_aligned = align.align_timestamps(timestamps_gt, positions_gt, quaternions_gt, timestamps_a)
    
    # position error
    delta_pos = positions_gt_aligned - positions_a
    ate_pos = np.sqrt(np.mean(np.sum(delta_pos ** 2, axis=1)))

    # rotation error
    delta_rot = [
        R.from_quat(q_a).inv() * R.from_quat(q_gt)
        for q_a, q_gt in zip(quaternions_a, quaternions_gt_aligned)
    ]
    ate_rot = np.mean([rot.magnitude() for rot in delta_rot])

    return ate_pos, ate_rot


def compute_cumulative_distances(positions):
    """compute cumulative distance of trajectory from positions"""
    deltas = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    return np.insert(np.cumsum(deltas), 0, 0)

def get_subtraj(dist, delta):
    """find subtrajectories with a given length (offset<0.5)"""
    index_map = {}
    pairs = []

    # use round()
    # so offset = 0.5

    for i, num in enumerate(dist):

        if round(num - delta) in index_map:
            pairs.append((index_map[round(num - delta)], i))

        if round(num + delta) in index_map:
            pairs.append((index_map[round(num + delta)], i))

        index_map[num] = i
    return pairs


def compute_rpe(timestamps_gt, positions_gt, quaternions_gt, timestamps_a, positions_a, quaternions_a, distances):
    """
    compute relative pose error (RPE) for positions and rotations

    Parameters
    ----------
    timestamps_gt, positions_gt, quaternions_gt : numpy.ndarray
        parameters from groundtruth
    timestamps_a, positions_a, quaternions_a : numpy.ndarray
        parameters from algorithm
    distances : list
        lengths of trajectories

    Returns
    -------
    rpe_pos : float
        Mean translational RPE.
    rpe_rot : float
        Mean rotational RPE in radians.
    """
    rpe_pos = []
    rpe_rot = []

    trajectories_gt_aligned, positions_gt_aligned, quaternions_gt_aligned = align.align_timestamps(timestamps_gt, positions_gt, quaternions_gt, timestamps_a)
    
    dist_gt = compute_cumulative_distances(positions_gt_aligned)
    dist_a = compute_cumulative_distances(positions_a)

    rpe_pos = np.zeros(len(distances))
    rpe_rot = np.zeros(len(distances))

    for d in distances:
        rpe_pos_d = []
        rpe_rot_d = []
        for start_idx, end_idx in get_subtraj(dist_a, d):
            # select gt subtrajectory
            delta_ts_gt = trajectories_gt_aligned[start_idx:end_idx]
            delta_pos_gt = positions_gt_aligned[start_idx:end_idx]
            delta_quat_gt = quaternions_gt_aligned[start_idx:end_idx]
            
            # select algorithm subtrajectory
            delta_ts_a = timestamps_a[start_idx:end_idx]
            delta_pos_a = positions_a[start_idx:end_idx]
            delta_quat_a = quaternions_a[start_idx:end_idx]
            
            # align algorithm subtrajectory to gt subtrajectory
            delta_pos_a, delta_quat_a = align. align_trajectories(delta_ts_gt, delta_pos_gt, delta_quat_gt, delta_ts_a, delta_pos_a, delta_quat_a)

            # calculate rpe position
            delta_pos_gt = positions_gt_aligned[end_idx] - positions_gt_aligned[start_idx]
            delta_pos_a = positions_a[-1] - positions_a[0]
            rpe_pos_d.append(np.linalg.norm(delta_pos_gt - delta_pos_a))

            # calculate rpe rotation
            rel_rot_gt = R.from_quat(quaternions_gt_aligned[start_idx]).inv() * R.from_quat(quaternions_gt_aligned[end_idx])
            rel_rot_a = R.from_quat(delta_quat_a[0]).inv() * R.from_quat(delta_quat_a[-1])
            delta_rot = rel_rot_gt.inv() * rel_rot_a
            rpe_rot_d.append(delta_rot.magnitude())


        rpe_pos[d - 1] = np.mean(np.mean(rpe_pos_d))
        rpe_rot[d - 1] = np.mean(np.mean(rpe_rot_d))

    return rpe_pos, rpe_rot