import numpy as np
from scipy.spatial.transform import Rotation as R


def match_groundtruth_to_timestamps(
    timestamps_gt, positions_gt, quaternions_gt, timestamps_a
):
    """
    align parameters of groundtruth trajectory with algorithm timestamps

    Parameters
    ----------
    timestamps_gt, positions_gt, quaternions_gt : numpy.ndarray
        parameters from groundtruth
    timestamps_a : numpy.ndarray
        timestamps from algorithm

    Returns
    -------
    aligned_positions_gt, aligned_quaternions_gt : numpy.ndarray
        aligned parameters of groundtruth trajectory
    """
    if len(timestamps_gt) == 0 or len(timestamps_a) == 0:
        raise ValueError("Timestamps cannot be empty.")
    if len(positions_gt) != len(timestamps_gt):
        raise ValueError("Mismatch between timestamps and positions lengths.")

    aligned_timestamps_gt = []
    aligned_positions_gt = []
    aligned_quaternions_gt = []

    for ts_a in timestamps_a:
        idx = np.argmin(np.abs(timestamps_gt - ts_a))
        aligned_timestamps_gt.append(timestamps_gt[idx])
        aligned_positions_gt.append(positions_gt[idx])
        aligned_quaternions_gt.append(quaternions_gt[idx])

    aligned_timestamps_gt = np.array(aligned_timestamps_gt)
    aligned_positions_gt = np.array(aligned_positions_gt)
    aligned_quaternions_gt = np.array(aligned_quaternions_gt)

    return aligned_timestamps_gt, aligned_positions_gt, aligned_quaternions_gt


def align_trajectory(
    timestamps_gt,
    positions_gt,
    quaternions_gt,
    timestamps_a,
    positions_a,
    quaternions_a,
):
    """
    align algorithm trajectory with groundtruth trajectory

    Parameters
    ----------
    timestamps_gt, positions_gt, quaternions_gt : numpy.ndarray
        parameters from groundtruth
    timestamps_a, positions_a, quaternions_a : numpy.ndarray
        parameters from algorithm

    Returns
    -------
    aligned_positions_a, aligned_quaternions_a : numpy.ndarray
        aligned parameters of algorithm trajectory
    """
    timestamps_gt, positions_gt, quaternions_gt = match_groundtruth_to_timestamps(
        timestamps_gt, positions_gt, quaternions_gt, timestamps_a
    )

    mu_gt = np.mean(positions_gt, axis=0)
    mu_a = np.mean(positions_a, axis=0)

    centered_gt = positions_gt - mu_gt
    centered_a = positions_a - mu_a

    n = np.shape(positions_gt)[0]

    # cross-covariance matrix
    C = np.dot(centered_gt.T, centered_a) / n

    U, _, Vt = np.linalg.linalg.svd(C)

    W = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt.T) < 0:
        W[2, 2] = -1

    delta_R = np.dot(U, np.dot(W, Vt))

    delta_t = mu_gt - np.dot(delta_R, mu_a)

    aligned_positions_a = np.dot(delta_R, positions_a.T).T + delta_t

    aligned_quaternions_a = []
    for q_gt, q_a in [*zip(quaternions_gt, quaternions_a)]:
        R_a = R.from_quat(q_a).as_matrix()
        R_aligned = np.dot(delta_R, R_a)
        q_aligned = R.from_matrix(R_aligned).as_quat()

        q_aligned /= np.linalg.norm(q_aligned)
        aligned_quaternions_a.append(q_aligned)

    aligned_quaternions_a = np.array(aligned_quaternions_a)

    aligned_quaternions_a /= np.linalg.norm(aligned_quaternions_a)

    return aligned_positions_a, aligned_quaternions_a
