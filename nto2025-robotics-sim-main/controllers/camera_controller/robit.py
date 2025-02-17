"""camera_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera
import numpy as np
import cv2

import socket
import threading
import pickle
import time
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

def create_color_mask(img, color_name):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    if color_name == 'green':
        lower = np.array([40, 50, 50])
        upper = np.array([80, 255, 255])
    elif color_name == 'yellow':
        lower = np.array([15, 50, 50])
        upper = np.array([40, 255, 255])
    elif color_name == 'white':
        lower = np.array([0, 0, 200])
        upper = np.array([180, 25, 255])
    elif color_name == 'black':
        lower = np.array([0, 0, 0])
        upper = np.array([180, 255, 30])
    elif color_name == 'gray':
        lower = np.array([0, 0, 100])
        upper = np.array([180, 30, 240])
    elif color_name == 'blu':
        lower = np.array([70, 50, 50])
        upper = np.array([120, 255, 255])
    elif color_name == 'red':
        lower1 = np.array([0, 100, 100])
        upper1 = np.array([10, 255, 255])
        lower2 = np.array([170, 100, 100])
        upper2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        return mask1 | mask2

    mask = cv2.inRange(hsv, lower, upper)
    return mask

def remove_small_areas(image, min_perimeter):
    contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    result_image = np.zeros_like(image)

    for contour in contours:
        perimeter = cv2.arcLength(contour, True)

        if perimeter >= min_perimeter:
            cv2.drawContours(result_image, [contour], -1, 255, thickness=cv2.FILLED)

    return result_image

def distance_2d(point1, point2):
    return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

def iEuler(data, x0, h):
    integral = np.zeros((data.shape[0] + 1, data.shape[1]))
    integral[0] = x0
    for i in range(1, integral.shape[0]):
        integral[i] = integral[i-1] + h*data[i-1]
    return integral

def iHeun(data, x0, h):
    data_ext = np.vstack((data[0], data))
    integral = np.zeros(data_ext.shape)
    integral[0] = x0
    for i in range(1, integral.shape[0]):
        integral[i] = integral[i-1] + h/2*(data_ext[i] + data_ext[i-1])
    return integral

def minimal_solution(r, b, h, phil0, phir0, coordinates, iSolver = None):

    d = distance_2d(coordinates[0], coordinates[1])
    v = d/h
    w = v/r
    print('WEED: ' + str(d))

    return [[w, w], [w, w]]

def draw_arrow(image_, x, y, direction, label):
    image_res = cv2.circle(image_, (int(x), int(y)), 3, (180, 255, 0), -1)

    image_res_text = cv2.putText(image_res, str(label), (int(x) + 10, int(y) + 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 255, 0), 1, cv2.LINE_AA)

    return image_res_text

def draw_arrows_on_image(img, coordinates):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    for i, coord in enumerate(coordinates):
        x, y, direction = coord
        image_res = draw_arrow(hsv, x, 600-y, (direction * (180 / math.pi)), i)

    return cv2.cvtColor(image_res, cv2.COLOR_BGR2RGB)

def get_rotate_speed(r, b, h, phi0):
    v = (phi0 * b) / (2 * r * h)
    return [v, -v]

def target_dest(dot, newDot, cur_psi):
    bearing = math.atan2(newDot[0] - dot[0], newDot[1] - dot[1])
    print("Ндо повернуть на:" + str(bearing - cur_psi) + ' из-за ' + str([newDot[0] - dot[0], newDot[1] - dot[1]]))
    e = bearing - cur_psi
    if e > np.pi:
        e = e - 2 * np.pi
    elif e < -np.pi:
        e = e + 2 * np.pi

    return e

def get_center_robot(img, id):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    detectorParams = cv2.aruco.DetectorParameters()

    detectorParams.adaptiveThreshWinSizeMin = 3
    detectorParams.adaptiveThreshWinSizeMax = 23
    detectorParams.adaptiveThreshConstant = 7
    detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)

    marker_corners, marker_ids, rejected_candidates = detector.detectMarkers(img)
    marker_id = np.array(marker_ids).flatten()
    for i in range(len(marker_id)):
        centr = (centre(marker_corners[i][0]))
        if marker_id[i] == id:
            start_center_robor2 = np.array(
                [centr[0], 600 - centr[1], (((ugol(centr, marker_corners[i][0])) - 360 ) * -1 )* (math.pi / 180)])
            return start_center_robor2


HOST = "127.0.0.1"
ROTO_1_PORT = 27001
ROTO_2_PORT = 27002

sock1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock1.connect((HOST, ROTO_1_PORT))

sock2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock2.connect((HOST, ROTO_2_PORT))

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
cam: Camera = robot.getDevice('camera')
cam.enable(timestep)

robot_1_desired_motor_speed = [0, 0]
robot_2_desired_motor_speed = [0, 0]
isRun = True
countStep = 1
countMove = 0
start_phi = 0
start_center_robor2 = np.array([0, 0, 0])

centr_robor1_start = [0, 0, 0]



# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:  # пока симуляция незавершена
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    data = cam.getImage()  # получаем рисунок в переенную
    # OpenCV image
    image = np.frombuffer(data, np.uint8).reshape(
        (cam.getHeight(), cam.getWidth(), 4))

    # sock1.sendall(pickle.dumps(robot_1_desired_motor_speed))
    # sock2.sendall(pickle.dumps(robot_2_desired_motor_speed))

    image_path = data
    if image is None:
        print(f"Error: Could not load image {image_path}")

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    detectorParams = cv2.aruco.DetectorParameters()

    # Настройка параметров детектора (попробуйте изменить эти значения)
    detectorParams.adaptiveThreshWinSizeMin = 3
    detectorParams.adaptiveThreshWinSizeMax = 23
    detectorParams.adaptiveThreshConstant = 7
    detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)

    marker_corners, marker_ids, rejected_candidates = detector.detectMarkers(
        image)  # координаты углов, значение маркера
    marker_id = np.array(marker_ids).flatten()
    # print(f"Markers found in image 11:",type (marker_id))
    for i in range(len(marker_id)):
        centr = (centre(marker_corners[i][0]))
        # centr_robor1_start = (centre(marker_corners[i][0]))
        # print(centr[0],centr[1],ugol(centr,marker_corners[i][0]))#пока не работает угол
        # print(marker_id[i],marker_corners[i],(centr[0],centr[1]))
        # print(marker_corners[i][0][3])
        if marker_id[i] == 10 and isRun:
            start_center_robor2 = np.array(
                [centr[0], 600-centr[1], (((ugol(centr, marker_corners[i][0])) - 360 ) * -1 )* (math.pi / 180)])
            print(f'Robot{marker_id[i] - 8}', marker_id[i], start_center_robor2)
        # Задаём фиксированное смещение (в пикселях)
        fixed_offset = 35

        # Создаём чёрно-белое изображение (одноканальное, тип uint8) такого же размера, как изображение с камеры
        bw_image = np.zeros((cam.getHeight(), cam.getWidth()), dtype=np.uint8)
        # Проверяем, что маркеры найдены, и преобразуем marker_ids в одномерный массив
        if marker_corners is not None and marker_ids is not None:
            marker_ids = np.array(marker_ids).flatten()
            # Обрабатываем каждый обнаруженный маркер
            for idx, corners in enumerate(marker_corners):
                # # Пропускаем маркеры с id 9 и 10
                # if marker_ids[idx] in [9, 10]:
                #     continue
                # Извлекаем точки углов (форма (4,2))
                pts = corners[0]
                # Вычисляем центр маркера
                center_pt = centre(pts)
                center_np = np.array(center_pt, dtype=np.float32)
                new_pts = []
                # Для каждого угла вычисляем смещённую точку
                for pt in pts:
                    pt_np = np.array(pt, dtype=np.float32)
                    # Вычисляем вектор от центра к углу
                    vec = pt_np - center_np
                    norm = np.linalg.norm(vec)
                    if norm > 0:
                        # Смещённый угол = центр + (направление вектора) * (текущее расстояние + fixed_offset)
                        new_pt = center_np + (vec / norm) * (
                                norm + fixed_offset)
                    else:
                        new_pt = pt_np
                    new_pts.append(new_pt.astype(np.int32))
                # Преобразуем список точек в массив нужной формы для cv2.polylines
                pts_array = np.array(new_pts).reshape((-1, 1, 2))
                # Рисуем контур (полилинию) на bw_image; цвет 255 (белый), толщина линии 2 пикселя
                cv2.polylines(bw_image, [pts_array], isClosed=True, color=255,thickness=2)

    image_blu = create_color_mask(image, 'blu')
    cleaned_image_blu = remove_small_areas(image_blu, 50)

    image_ye = create_color_mask(image, 'yellow')
    cleaned_image_ye = remove_small_areas(image_ye, 50)

    image_gray = create_color_mask(image, 'gray')
    cleaned_image_gray = remove_small_areas(image_gray, 150)

    # image_green = create_color_mask(image, 'green')

    # cv2.imshow('Start', image)
    cv2.imshow('Adjusted Squares', bw_image)
    # cv2.imshow('Blu zone', cleaned_image_blu)
    # cv2.imshow('Ye zone', cleaned_image_ye)
    # cv2.imshow('Gray zone', cleaned_image_gray)
    # cv2.imshow('Green zone', image_green)
    key = cv2.waitKey(1)

    if key == ord('q'):
        break

    coordinates = np.empty((8, 3))

    coordinates[0] = start_center_robor2
    coordinates[1] = np.array([408, 600-430, 0])
    coordinates[2] = np.array([408, 600-350, 0])
    coordinates[3] = np.array([408, 600-270, 0])

    coordinates[4] = np.array([340, 600 - 200, 0])
    coordinates[5] = np.array([280, 600 - 200, 0])

    coordinates[6] = np.array([200, 600 - 270, 0])
    coordinates[7] = np.array([200, 600-350, 0])
    np.set_printoptions(precision=2, suppress=True)

    image_with_arrows = draw_arrows_on_image(image, coordinates)
    cv2.imshow('Arrow', image_with_arrows)

    if (isRun):
        isRun = False
        start_phi = start_center_robor2[2]
        print(start_center_robor2)
        print(coordinates)
        # sol = minimal_solution(5, 50, 1, 0, 0, coordinates)
        # print(sol)
        # set_motor_speeds(sol, timestep )

    if (isRun == False):
        if (countStep == timestep * 1):
            countStep = 1


            if (countMove < len(coordinates)-1):
                print('=========' + str(countMove+1) + '=========')
                print("Mu dot: \n" + str(coordinates[countMove:countMove + 2]))
                sol = minimal_solution(5, 50, 1, 0, 0, coordinates[countMove:countMove + 2])

                sock2.sendall(pickle.dumps(sol[0]))
                print(f"Setting motor speeds to: {sol[0]}")

                if(countMove < len(coordinates)-2):
                    phi = target_dest(coordinates[countMove + 1], coordinates[countMove + 2], start_phi)
                    print("Rotate dot: "+ str(int(phi * (180 / math.pi))) + ':' +str(int(start_phi * (180 / math.pi))) + str(coordinates[countMove + 1]) + ':' + str( coordinates[countMove + 2]))
                    if(phi != 0):
                        start_phi += phi
                        print('Center: ' + str(get_center_robot(image, 10)[2]* (180 / math.pi)))
                        speed = get_rotate_speed(5, 50, 1, phi)
                        print(speed)
                        stamp = time.time()
                        while robot.step(timestep) != -1 and time.time() - stamp < 1:
                            pass
                        sock2.sendall(pickle.dumps(speed))
                        print('ROTATE finishe: ' + str(start_phi))
                        stamp = time.time()
                        while robot.step(timestep) != -1 and time.time() - stamp < 1.1:
                            pass
                        sock2.sendall(pickle.dumps([0, 0]))

                countMove += 1

            else:
                sock2.sendall(pickle.dumps([0, 0]))

        else:
            countStep += 1