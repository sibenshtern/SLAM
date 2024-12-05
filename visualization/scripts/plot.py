import open3d as o3d
import numpy as np
import colorsys
import argparse

from scipy.spatial.transform import Rotation as R

import load
import align


def draw_trajectory(positions, quaternions, color=[1, 0, 0]):
    """
    draw single trajectory using open3d

    Parameters
    ----------
    positions, quaternions : numpy.ndarray
    color : list, default=[1, 0, 0]

    Returns
    -------
    frames : list
    """
    frames = []

    for i, (pos, quat) in enumerate(zip(positions, quaternions)):
        size = 0.08

        np_points = np.array([[0, 0, 0], [size, 0, 0], [0, size, 0], [0, 0, size]])
        np_lines = np.array([[0, 1], [0, 2], [0, 3]])

        l = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(np_points),
            lines=o3d.utility.Vector2iVector(np_lines),
        )

        rot = rot = R.from_quat(quat).as_matrix()

        l.paint_uniform_color(color)
        l.rotate(rot, center=(0, 0, 0))
        l.translate(pos)

        frames.append(l)

    return frames


def plot_from_tpq(args):
    """
    plot (single / multiple) trajectories from timestamps, positions and quaternions

    Parameters
    ----------
    timestamps, positions, quaternions : numpy.ndarray
    """
    frames = []
    colors = [
        colorsys.hsv_to_rgb(i / len(args.positions), 1, 1)
        for i in range(len(args.timestamps))
    ]

    # find shortest trajectory
    idx_min = np.argmin([len(ts) for ts in args.timestamps])

    for i in range(len(args.timestamps)):
        ts, p, q = align.match_groundtruth_to_timestamps(
            args.timestamps[i],
            args.positions[i],
            args.quaternions[i],
            args.timestamps[idx_min],
        )
        frames += draw_trajectory(p, q, colors[i])

    o3d.visualization.draw_geometries(frames)


def plot_from_filepaths(args):
    """
    plot (single / multiple) trajectories from filepaths

    Parameters
    ----------
    filepath : str
    """
    # gt
    t_gt, p_gt, q_gt = load.load_trajectory(args.filepaths[0])

    # qw, qx, qy, qz -> qx, qy, qz, qw
    q_gt = q_gt[:, [1, 2, 3, 0]]

    timestamps = [t_gt]
    positions = [p_gt]
    quaternions = [q_gt]

    for i, f in enumerate(args.filepaths[1:]):
        t_a, p_a, q_a = load.load_trajectory(f)
        p_a, q_a = align.align_trajectory(t_gt, p_gt, q_gt, t_a, p_a, q_a)
        timestamps.append(t_a)
        positions.append(p_a)
        quaternions.append(q_a)

    plot_from_tpq(
        argparse.Namespace(
            timestamps=timestamps, positions=positions, quaternions=quaternions
        )
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="plot.py")

    parser.add_argument("filepaths", nargs="+", help="paths to the trajectories")

    args = parser.parse_args()

    plot_from_filepaths(args)
