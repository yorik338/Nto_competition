import math as m
from json import dumps
import socket
import pickle

import numpy as np


class RobotMoto():
    def __init__(self, ip, port, coords, angle) -> None:
        # Коэффициенты регуляторов
        self.K1 = 5.2
        self.K2 = 0.2
        # Коэффициент пропорционального регулятора
        self.Kp = 10
        self.Ki = 0
        self.Kd = 0
        self.Isum = 0
        self.Kp_t = 10
        self.Ki_t = 0.1
        self.Kd_t = 0
        self.Isum_t = 0
        self.Kp_r = 10
        self.Ki_r = 0.1
        self.Kd_r = 0
        self.Isum_r = 0
        self.IU = 0.01
        # Динстанция до точки назначения, при которой она считается достигнутой
        self.dest_radius1 = 7
        # Динстанция до очередной точки, при которой она считается достигнутой
        self.dest_radius2 = 10
        # Допустимый разброс достигнутого курса робота (в радианах)
        self.course_scat1 = 0.2
        self.course_scat2 = 0.2
        # Расстояние в точках до точки назначения с которых начинается замедление
        self.Dd = 3
        # Коэффициент замедления перед точкой назначения
        self.Ks = 0.05
        self.cur_x, self.cur_y = coords
        self.cur_psi = angle
        self.prev_psi = 0
        # Точки назначения движения
        self.dests = []
        # Настоящая точка назначения движения робота
        self.dest = 0
        self.UDP_IP = ip
        self.UDP_PORT = port
        self.Usum = 0
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.UDP_IP, self.UDP_PORT))
        except SerialException:
            print('[ERROR] Bad serial port')
            quit()

    #
    # Управление двигателями
    #
    def set_control_bt(self, left, right):
        if left < -255:
            left = -255
        if left > 255:
            left = 255
        if right < -255:
            right = -255
        if right > 255:
            right = 255
        self.serial.write(dumps({
            "left": int(left),
            "right": int(right),
        }).encode())

    def set_control(self, left, right):
        if left < -20:
            left = -20
        if left > 20:
            left = 20
        if right < -20:
            right = -20
        if right > 20:
            right = 20
        print('Soket go: ' + str([int(left), int(right)]))
        self.sock.sendall(pickle.dumps([int(left), int(right)]))
        # s.sendto(msg, (self.UDP_IP, self.UDP_PORT))

    #
    # Работа с точками назначения
    #
    def set_dests(self, dest: list) -> None:
        self.dests.extend(dest)

    def next_dest(self) -> bool:
        if self.dest == len(self.dests) - 1:
            return False
        self.dest += 1
        return True

    def around_dest(self) -> bool:
        dest_x, dest_y = self.dests[self.dest][0]
        if self.dest != len(self.dests) - 1:
            radius = self.dest_radius2
        else:
            radius = self.dest_radius1
        dist_x = abs(self.cur_x - dest_x)
        dist_y = abs(self.cur_y - dest_y)
        # print(dist_x**2 + dist_y**2)
        return dist_x ** 2 + dist_y ** 2 < radius ** 2

    def around_course(self) -> bool:
        req_psi = self.dests[self.dest][1]
        if req_psi == None:
            return True
        e = req_psi - self.cur_psi
        if e > np.pi:
            e -= 2 * np.pi
        elif e < -np.pi:
            e += 2 * np.pi
        return -self.course_scat1 < e and e < self.course_scat1

    def is_targeted(self) -> bool:
        if self.dest == len(self.dests):
            return True
        dest_x, dest_y = self.dests[self.dest][0]
        bearing = m.atan2(dest_y - self.cur_y, dest_x - self.cur_x)
        e = bearing - self.cur_psi
        if e > np.pi:
            e -= 2 * np.pi
        elif e < -np.pi:
            e += 2 * np.pi
        return -self.course_scat2 < e and e < self.course_scat2

    #
    # Регуляторы движения
    #
    def reg_lyap(self):
        dest_x, dest_y = self.dests[self.dest][0]
        distance = m.sqrt((dest_x - self.cur_x) ** 2 + (dest_y - self.cur_y) ** 2)
        bearing = m.atan2(dest_y - self.cur_y, dest_x - self.cur_x)
        courseAngle = bearing - self.cur_psi
        if courseAngle > np.pi:
            courseAngle = courseAngle - 2 * np.pi
        if courseAngle < -np.pi:
            courseAngle = courseAngle + 2 * np.pi
        ul = self.K1 * np.cos(courseAngle) * \
             np.tanh(distance) + self.K2 * (courseAngle)
        ur = self.K1 * np.cos(courseAngle) * \
             np.tanh(distance) - self.K2 * (courseAngle)
        return [ul, ur]

    def reg_lpid(self):
        dest_x, dest_y = self.dests[self.dest][0]
        stopping = self.Ks if self._points_to_dest() < self.Dd else 1
        distance = m.sqrt((dest_x - self.cur_x) ** 2 + (dest_y - self.cur_y) ** 2)
        bearing = m.atan2(dest_y - self.cur_y, dest_x - self.cur_x)
        courseAngle = bearing - self.cur_psi
        if courseAngle > np.pi:
            courseAngle = courseAngle - 2 * np.pi
        if courseAngle < -np.pi:
            courseAngle = courseAngle + 2 * np.pi
        # if courseAngle > 0:
        #     courseAngle -= np.pi
        # elif courseAngle < 0:
        #     courseAngle -= np.pi
        # print(courseAngle)
        # print()
        # return [0,0]
        e = courseAngle * self.Kp + (self.cur_psi - self.prev_psi) * self.Kd \
            + (self.Isum + courseAngle) * self.Ki
        u = self.K1 * np.cos(courseAngle) * \
            np.tanh(distance)
        # np.tanh(distance) + self.Usum * self.IU

        self.Usum += u
        ul = u - e
        ur = u + e
        # print(e)
        self.Isum += courseAngle
        self.prev_psi = self.cur_psi
        return [ul, ur]

    def reg_pid(self):
        req_psi = self.dests[self.dest][1]
        if req_psi == None:
            return [0, 0]
        e = req_psi - self.cur_psi
        # print(e)
        if e > np.pi:
            e -= 2 * np.pi
        if e < -np.pi:
            e += 2 * np.pi
        # print(e)
        # if e > 0:
        #     e -= np.pi
        # elif e < 0:
        #     e += np.pi
        u = self.Kp_r * e + (self.Isum_r + e) * self.Ki_r \
            - (self.cur_psi - self.prev_psi) * self.Kd_r
        self.Isum_r += e
        self.prev_psi = self.cur_psi
        return [-u, u]

    def target_dest(self):
        dest_x, dest_y = self.dests[self.dest][0]
        bearing = m.atan2(dest_x - self.cur_x, dest_y - self.cur_y)
        e = bearing - self.cur_psi
        if e > np.pi:
            e = e - 2 * np.pi
        elif e < -np.pi:
            e = e + 2 * np.pi
        # if e > 0:
        #     e -= np.pi
        # elif e < 0:
        #     e += np.pi
        # print(e)
        u = self.Kp_t * e + (self.Isum_t + e) * self.Ki_t \
            - (self.cur_psi - self.prev_psi) * self.Kd_t
        self.Isum_t += e
        self.prev_psi = self.cur_psi
        # u = self.Kp_t * e
        return [-u, u]

    #
    # Обновление состояния робота
    #
    def set_pos(self, coords, angle) -> None:
        self.cur_x, self.cur_y = coords
        self.cur_psi = angle

    def debug(self):
        ret = ''
        ret += f'Robot coords X:{self.cur_x} Y: {self.cur_y}\n'
        ret += f'Robot current course:{round(self.cur_psi, 6)}\n'
        if self.dests == []:
            ret += 'No destination\n'
        else:
            req_psi = self.dests[self.dest][1]
            dest_x, dest_y = self.dests[self.dest][0]
            ret += f'Robot destination coords X:{dest_x} Y: {dest_y}\n'
            if req_psi != None:
                ret += f'Robot requred course:{round(req_psi, 6)}\n'
        print(ret)

    #
    # Other
    #
    def dump_point(self) -> None:
        point = self.dests[self.dest][2]
        if point != None and point != 'Start':
            return point + ' OK'
        return None

    def _points_to_dest(self):
        return len(self.dests) - self.dest
