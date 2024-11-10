import numpy as np
import cv2
import glob
import sys


def get_parameters_from_yaml(file_name, parameters):
    def str_to_list(string: str) -> list:
        return list(map(float, string.strip('[]').split(', ')))


    with open(file_name, mode='r') as file:
        data = {}

        current_cam = None
        for line in file:
            if line.startswith("cam"):
                current_cam = line.strip(':\n')
                data[current_cam] = {}

            fixed_line = [obj.strip() for obj in line.strip().split(': ')]
            if fixed_line[0] in parameters:
                value = fixed_line[1]
                if fixed_line[1].startswith('['):
                    value = str_to_list(fixed_line[1])

                data[current_cam][fixed_line[0]] = value

    return data


def undistortion(cam_number):
    args = sys.argv # img_0, img_1, yaml_file

    img = cv2.imread(args[cam_number+1])
    h,  w = img.shape[:2]

    calibration_coefficients = get_parameters_from_yaml(args[3], ['distortion_coeffs', 'intrinsics'])

    dist = np.array(calibration_coefficients[f'cam{cam_number}']['distortion_coeffs'])
    f_x, f_y, c_x, c_y = calibration_coefficients[f'cam{cam_number}']['intrinsics']

    camera_matrix = np.array([[f_x, 0.0, c_x],  [0.0, f_y, c_y], [0.0, 0.0, 1.0]])

    newcameramatrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist, (w, h), 1, (w, h))
    undistorted_image = cv2.undistort(img, camera_matrix, dist, None, newcameramatrix)
    cv2.imwrite(f'img{cam_number}_undistortion.png', undistorted_image)

undistortion(0)
undistortion(1)
