import numpy as np
import argparse
import os.path

from tabulate import tabulate

import load
import align
import errors

import plot


def unpack(args):
    """unpack trajectories from filepaths"""
    timestamps = []
    positions = []
    quaternions = []

    for a in [args.gt] + args.alg:
        t_a, p_a, q_a = load.load_trajectory(a, flag_ts=True)

        timestamps.append(t_a)
        positions.append(p_a)
        quaternions.append(q_a)

    return timestamps, positions, quaternions

def transform(args):
    """align algorithm trajectories with groundtruth trajectory"""
    t_gt = args.timestamps[0]
    p_gt = args.positions[0]
    q_gt = args.quaternions[0]

    aligned_p = []
    aligned_q = []

    aligned_p.append(p_gt)
    aligned_q.append(q_gt)

    for (t_a, p_a, q_a) in [*zip(args.timestamps[1:], args.positions[1:], args.quaternions[1:])]:
        aligned_p_a, aligned_q_a = align.align_trajectories(t_gt, p_gt, q_gt, t_a, p_a, q_a)

        aligned_p.append(aligned_p_a)
        aligned_q.append(aligned_q_a)

    return aligned_p, aligned_q


if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="analysis.py")

    parser.add_argument("gt", nargs="?", help="path to groundtruth trajectory")
    parser.add_argument("alg", nargs="*", help="paths to algorythms trajectories")
    parser.add_argument("-f", "--frequency", nargs='?', type=int, default=30,
                        help="frequency of plotted trajectories")

    parser.add_argument("-c", "--compute", action="store_true")
    parser.add_argument("-p", "--plot", action="store_true")
    args = parser.parse_args()

    timestamps, positions, quaternions = unpack(argparse.Namespace(gt=args.gt, alg=args.alg))
    positions, quaternions = transform(argparse.Namespace(timestamps=timestamps, positions=positions, quaternions=quaternions))
 
    if args.plot:
        plot.plot_from_pq(argparse.Namespace(positions=positions, quaternions=quaternions, frequency=args.frequency))

    if args.compute:
        headers = ["Trajectory", "ATE Pos", "ATE Rot", "RPE Pos (d=1)", "RPE Rot (d=1)"]
        results = [[os.path.basename(args.gt), "0.0", "0.0", "0.0", "0.0"]]

        for i in range(1, len(timestamps)):
            ate_pos, ate_rot = errors.compute_ate(timestamps[0], positions[0], quaternions[0], timestamps[i], positions[i], quaternions[i])

            max_distance = 2
            distances = range(1, max_distance)

            rpe_pos, rpe_rot = errors.compute_rpe(timestamps[0], positions[0], quaternions[0], timestamps[i], positions[i], quaternions[i], distances)

            results.append([os.path.basename(args.alg[i - 1]), ate_pos, ate_rot, rpe_pos[0], rpe_rot[0]])

        print(tabulate(results, headers=headers, floatfmt=".6f"))
        
     
        
