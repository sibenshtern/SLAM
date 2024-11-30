import open3d as o3d
import numpy as np
import colorsys
import argparse

from scipy.spatial.transform import Rotation as R

import load
import align


def draw_trajectory(positions, quaternions, frequency=30, color=[1, 0, 0]):
    """
    draw single trajectory using open3d

    Parameters
    ----------
    positions, quaternions : numpy.ndarray
    frequency : int, default=30
    color : list, default=[1, 0, 0]

    Returns
    -------
    frames : list
    """
    frames = []

    for i, (pos, quat) in enumerate(zip(positions[::frequency], quaternions[::frequency])):
        #f = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)

        size = 0.2

        np_points = np.array([[0, 0, 0], [size, 0, 0], [0, size, 0], [0, 0, size]])
        np_lines = np.array([[0, 1], [0, 2], [0, 3]])

        l = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(np_points),
            lines=o3d.utility.Vector2iVector(np_lines)
        )

        rot = rot = R.from_quat(quat).as_matrix()

        l.paint_uniform_color(color)
        l.rotate(rot, center=(0, 0, 0))
        l.translate(pos)

        frames.append(l)

    return frames

def plot_from_filepaths(args):
    """
    plot (single / multiple) trajectories from filepaths

    Parameters
    ----------
    filepath : str
    frequency : int, default=30
    """
    frames = []
    colors = [colorsys.hsv_to_rgb(i / len(args.filepaths), 1, 1) for i in range(len(args.filepaths))]

    # gt
    t_gt, p_gt, q_gt = load.load_trajectory(args.filepaths[0])
    frames += draw_trajectory(p_gt, q_gt, args.frequency, colors[-1])
    
    for i, f in enumerate(args.filepaths[1:]):
        timestamps, positions, quaternions = load.load_trajectory(f)
        positions, quaternions = align.align_trajectories(t_gt, p_gt, q_gt, timestamps, positions, quaternions)
        frames += draw_trajectory(positions, quaternions, args.frequency, colors[i])

    o3d.visualization.draw_geometries(frames)


def plot_from_pq(args):
    """
    plot (single / multiple) trajectories from positions and quaternions

    Parameters
    ----------
    positions, quaternions : list
    frequency : int, default=30
    """
    frames = []
    colors = [colorsys.hsv_to_rgb(i / len(args.positions), 1, 1) for i in range(len(args.positions))]

    for i, (pos, quat) in enumerate([*zip(args.positions, args.quaternions)]):
        frames += draw_trajectory(pos, quat, args.frequency, colors[i])

    o3d.visualization.draw_geometries(frames)
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="plot.py")

    parser.add_argument("filepaths", nargs="+", help="paths to the trajectories")
    parser.add_argument("-f", "--frequency", nargs='?', type=int, default=30,
                        help="frequency of plotted trajectories")

    args = parser.parse_args()

    plot_from_filepaths(args)