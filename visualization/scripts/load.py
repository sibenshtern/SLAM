import pandas as pd
import numpy as np


def load_trajectory(filepath, flag_ts=True):
    """
    load trajectories from file

    Parameters
    ----------
    filepath : .csv or .txt file

    Returns
    -------
    timestamps, positions, quaternions : numpy.ndarray

    Raises
    ------
    Unsupported file format
        if filepath is not .csv or .txt format
    """
    if filepath.endswith(".csv"):
        df = pd.read_csv(filepath)

        timestamps = df.iloc[:, 0].values
        positions = df.iloc[:, 1:4].values
        quaternions = df.iloc[:, 4:8].values

    elif filepath.endswith(".txt"):
        timestamps = []
        positions = []
        quaternions = []

        with open(filepath, "r") as file:
            for line in file:
                values = line.strip().split()
                timestamps.append(float(values[0]))
                positions.append([float(v) for v in values[1:4]])
                quaternions.append([float(v) for v in values[4:]])

        timestamps = np.array(timestamps)
        positions = np.array(positions)
        quaternions = np.array(quaternions)

    else:
        raise ValueError("Unsupported file format")

    if flag_ts:
        return timestamps, positions, quaternions
    else:

        return positions, quaternions
