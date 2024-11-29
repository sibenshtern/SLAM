This scripts helps analyzing and comparing trajectory data. It supports:
- Loading and preprocessing of trajectory data.
- Alignment of algorithm-generated trajectories to ground truth.
- Computation of trajectory errors (ATE and RPE).
- Visualization of 3D trajectories using `Open3D`.

## Dependencies:
```bash
pip install numpy scipy pandas tabulate open3d
```

# Usage
## 1. Analyze Trajectories
Run analysis.py to compute errors and optionally visualize trajectories:
```bash
python analysis.py <gt_filepath> <alg_filepath1> <alg_filepath2> ... -c -p
```
## Arguments:
- `<gt_filepath>`: Path to the ground truth trajectory file.
- `<alg_filepathX>`: Paths to algorithm-generated trajectory files.
- `-c`, `--compute`: Compute error metrics.
- `-p`, `--plot`: Plot trajectories.
- `-f <frequency>`, `--frequency <frequency>`: Sampling frequency for visualization (default: 30).

## 2. Plot Trajectories
Use plot.py to visualize one or more trajectories:
```bash
python plot.py <trajectory_filepath1> <trajectory_filepath2> ... -f 10
```
## Arguments:
- `<trajectory_filepathX>`: Paths to trajectory files.
- `-f <frequency>`, `--frequency <frequency>`: Sampling frequency for visualization (default: 30).

# Output:
## 1. Error Metrics: Console table of ATE and RPE.
```
Trajectory              ATE Pos    ATE Rot    RPE Pos (d=1)    RPE Rot (d=1)
--------------------  ---------  ---------  ---------------  ---------------
data.csv               0.000000   0.000000         0.000000         0.000000
f_MH03.txt.txt         0.028140   2.940068         1.023401         0.091778
data_MH03_medium.csv   3.707404   0.958546         0.518286         0.055737
```

## (optional) 2. Plots: Interactive 3D plots of trajectories
![Imgur](https://imgur.com/JflGtp1.jpg)
