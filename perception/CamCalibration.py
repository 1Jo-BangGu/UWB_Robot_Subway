# CamCalibration.py
import cv2
import numpy as np
import os

# 현재 파일의 경로를 기준으로 .npy 경로 설정
this_dir = os.path.dirname(__file__)
K = np.load(os.path.join(this_dir, "camera_matrix.npy"))
dist = np.load(os.path.join(this_dir, "dist_coeffs.npy"))
# crop 좌표 (네가 실험해서 고정한 값)
#crop_x, crop_y, crop_w, crop_h = 118, 72, 1144, 506

# 첫 프레임으로 new_K 계산 (frame은 main에서 전달받음)
def get_new_K(frame):
    h, w = frame.shape[:2]
    new_K, _ = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))
    return new_K

# 보정 + crop 함수
def undistort_frame(frame, new_K):
    undistorted = cv2.undistort(frame, K, dist, None, new_K)
    #cropped = undistorted[crop_y:crop_y+crop_h, crop_x:crop_x+crop_w]
    return undistorted
