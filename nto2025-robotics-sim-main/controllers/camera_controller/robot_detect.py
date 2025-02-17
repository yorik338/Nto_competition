from typing import List, Sequence, Tuple, Union

import cv2
import numpy as np
import math

def centre(tochki):
    x_coor = [point[0] for point in (tochki)]
    y_coor = [point[1] for point in (tochki)]
    centr_x = int(np.mean(x_coor))
    centr_y = int(np.mean(y_coor))
    return [centr_x, centr_y]

def ugol(centre, tochki):
    t3x, t3y = tochki[0]
    t4x, t4y = tochki[1]
    t34 = ((t3x + t4x) / 2, (t3y + t4y) / 2)
    delta_x = t34[0] - centre[0]
    delta_y = t34[1] - centre[1]
    angle_rad = np.arctan2(delta_y, delta_x)
    angle_deg = np.degrees(angle_rad)
    angle_deg = (angle_deg + 360) % 360
    return angle_deg

def get_orientation(image, id):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    detectorParams = cv2.aruco.DetectorParameters()

    detectorParams.adaptiveThreshWinSizeMin = 3
    detectorParams.adaptiveThreshWinSizeMax = 23
    detectorParams.adaptiveThreshConstant = 7
    detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)

    marker_corners, marker_ids, rejected_candidates = detector.detectMarkers(image)
    marker_id = np.array(marker_ids).flatten()

    for i in range(len(marker_id)):
        centr = (centre(marker_corners[i][0]))
        if marker_id[i] == id:
            start_center_robor2 = np.array(
                [centr[0], 600 - centr[1], (((ugol(centr, marker_corners[i][0])) - 360)) * (math.pi / 180)])
            return start_center_robor2