This script is designed for automatic calibration and obtaining undistortion using the framework Kalibr and OpenCV tools.

To run calibration script use the command 
```bash
./calibrate.sh /path_to_directory bag_file yaml_file model_cam1 model_cam2 topic1 topic2
```

For the script to work correctly, use the following structure
```
workspace/
├─ data/
│  ├─ pictures/
│  ├─ bag_file.bag
│  └─ yaml_file.yaml
├─ scripts/
│  ├─ import_pictures_for_undistortion.py
│  ├─ parse_yaml.py
│  ├─ undistortion_equi.py
│  └─ undistortion_radtan.py
├─ calibrate.sh
└─ kalibr_inside.sh
```
