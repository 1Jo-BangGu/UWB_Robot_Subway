import cv2
import numpy as np
from vision.perception import detect_robot_position
from planning.planner import a_star, find_nearest_goal, draw_path_on_frame
from control.controller import follow_path
from network.client import RaspiVisionClient
from vision.CamCalibration import undistort_frame, get_new_K

world_points = np.array([[0, 0], [0, 90], [180, 90], [180, 0]], dtype=np.float32)
clicked_points = []
obstacle_clicks = []
goal_clicks = []
M_pixel2real = None
M_real2pixel = None
transform_calculated = False
obstacle_done = False
goal_done = False

client = RaspiVisionClient("192.168.201.168")
client.start()
while client.latest_frame is None:
    pass
new_K = get_new_K(client.latest_frame)
cv2.namedWindow("Robot View")

def mouse_callback(event, x, y, flags, param):
    global M_pixel2real, M_real2pixel, transform_calculated
    if event == cv2.EVENT_LBUTTONDOWN:
        if not transform_calculated and len(clicked_points) < 4:
            clicked_points.append([x, y])
            if len(clicked_points) == 4:
                M_pixel2real = cv2.getPerspectiveTransform(np.array(clicked_points, dtype=np.float32), world_points)
                M_real2pixel = np.linalg.inv(M_pixel2real)
                transform_calculated = True
        elif transform_calculated and not obstacle_done:
            obstacle_clicks.append([x, y])
        elif transform_calculated and obstacle_done and not goal_done:
            goal_clicks.append([x, y])

cv2.setMouseCallback("Robot View", mouse_callback)

while True:
    with client.queue_lock:
        frame = client.latest_frame.copy() if client.latest_frame is not None else None
    if frame is None:
        continue
    frame = undistort_frame(frame, new_K)

    for pt in clicked_points:
        cv2.circle(frame, tuple(pt), 6, (0, 0, 255), -1)

    obstacle_list = []
    if M_pixel2real is not None:
        for pt in obstacle_clicks:
            px = np.array([[[pt[0], pt[1]]]], dtype=np.float32)
            real = cv2.perspectiveTransform(px, M_pixel2real)[0][0]
            obstacle_list.append((real[0], real[1], 10))

    goal_list = []
    if M_pixel2real is not None:
        for pt in goal_clicks:
            px = np.array([[[pt[0], pt[1]]]], dtype=np.float32)
            real = cv2.perspectiveTransform(px, M_pixel2real)[0][0]
            goal_list.append((real[0], real[1]))

    key = cv2.waitKey(1)
    if key == ord('d'):
        obstacle_done = True
    elif key == ord('g'):
        goal_done = True
    elif key == 27:
        break

    if transform_calculated and obstacle_done and goal_done:
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        robot_pose, screen_pos = detect_robot_position(frame, aruco_dict, parameters, M_pixel2real)
        if robot_pose and goal_list:
            path = a_star((robot_pose[0], robot_pose[1]), find_nearest_goal((robot_pose[0], robot_pose[1]), goal_list), (0,180,0,90), obstacle_list)
            follow_path(robot_pose, path)
            frame = draw_path_on_frame(frame, path, M_real2pixel)

    cv2.imshow("Robot View", frame)
cv2.destroyAllWindows()
