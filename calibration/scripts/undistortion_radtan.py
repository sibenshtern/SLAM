import numpy as np
import cv2
import sys
from parse_yaml import get_parameters_from_yaml


def undistortion(cam_number):
    args = sys.argv # img_0, img_1, yaml_file, path_to_result

    img = cv2.imread(args[cam_number+1])
    h,  w = img.shape[:2]

    calibration_coefficients = get_parameters_from_yaml(args[3], ['distortion_coeffs', 'intrinsics'])

    dist = np.array(calibration_coefficients[f'cam{cam_number}']['distortion_coeffs'])
    f_x, f_y, c_x, c_y = calibration_coefficients[f'cam{cam_number}']['intrinsics']
    
    camera_matrix = np.array([[f_x, 0.0, c_x],  [0.0, f_y, c_y], [0.0, 0.0, 1.0]])
    
    newcameramatrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist, (w, h), 1, (w, h))
    undistorted_image = cv2.undistort(img, camera_matrix, dist, None, newcameramatrix)
    cv2.imwrite(f'{args[4]}_undistortion_radtan.png', undistorted_image)


if __name__ == "__main__":
    undistortion(0)
    undistortion(1)
