import numpy as np
import cv2
import sys
from parse_yaml import get_parameters_from_yaml


FOV = 125
HEIGHT = 1000

class Data:
    def __init__(self, cam_n, path_img, path_yaml):
        self.img = cv2.imread(path_img)
        self.h, self.w = self.img.shape[:2]
        self.calibration_coefficients = get_parameters_from_yaml(path_yaml, ['distortion_coeffs', 'intrinsics'])
        self.dist = np.array(self.calibration_coefficients[f'cam{cam_n}']['distortion_coeffs'])
        f_x, f_y, c_x, c_y = self.calibration_coefficients[f'cam{cam_n}']['intrinsics']
        self.camera_matrix = np.array([[f_x, 0.0, c_x],  [0.0, f_y, c_y], [0.0, 0.0, 1.0]])


def undistortion():
    args = sys.argv # img_0, img_1, yaml_file, path_to_result

    img_l = Data(0, args[1], args[3])
    img_r = Data(0, args[2], args[3])
    window_size = 5
    min_disp = 0
    num_disp = 112 - min_disp
    max_disp = min_disp + num_disp
    
    stereo_fov_rad = FOV * (np.pi/180)
    stereo_height_px = HEIGHT
    stereo_focal_px = stereo_height_px/2 / np.tan(stereo_fov_rad/2)
    stereo_width_px = stereo_height_px + max_disp
    stereo_size = (stereo_width_px, stereo_height_px)
    stereo_cx = (stereo_height_px - 1)/2 + max_disp
    stereo_cy = (stereo_height_px - 1)/2
    
    P_left = np.array([[stereo_focal_px, 0, stereo_cx, 0],
                       [0, stereo_focal_px, stereo_cy, 0],
                       [0,               0,         1, 0]])
    P_right = P_left.copy()
    
    (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(img_l.camera_matrix, img_l.dist, np.eye(3), P_left, stereo_size, cv2.CV_32FC1)
    (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(img_r.camera_matrix, img_r.dist, np.eye(3), P_right, stereo_size, cv2.CV_32FC1)
    undistort_rectify = {"left"  : (lm1, lm2),
                         "right" : (rm1, rm2)}
    
    center_undistorted = {"left" : cv2.remap(src = img_l.img,
                                          map1 = undistort_rectify["left"][0],
                                          map2 = undistort_rectify["left"][1],
                                          interpolation = cv2.INTER_LINEAR),
                                  "right" : cv2.remap(src = img_r.img,
                                          map1 = undistort_rectify["right"][0],
                                          map2 = undistort_rectify["right"][1],
                                          interpolation = cv2.INTER_LINEAR)}

    image = cv2.cvtColor(center_undistorted["left"][:, max_disp:], cv2.COLOR_BGR2RGB)

    cv2.imwrite(f'{args[4]}/img_undistortion_equi.png', image)


if __name__ == "__main__":
    undistortion()