from json import dumps
import numpy as np

import socket
import pickle
import math


class RobotMoto():
    def __init__(self, ip, port, coords, angle, id, isLog=False):
        self.ID = id
        self.IP = ip
        self.PORT = port
        self.cur_x, self.cur_y = coords
        self.cur_psi = angle
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.IP, self.PORT))
        except SerialException:
            print('[ERROR] Bad serial port')
            quit()
        self.isLog = isLog
        self.teg = '[ROBOT ' + str(self.ID - 8) + '] '
        if(self.isLog):
            print(self.teg + 'Create robot' )
    
    def set_pos(self, coords, angle) -> None:
        self.cur_x, self.cur_y = coords
        self.cur_psi = angle
        if(self.isLog):
            print(self.teg + 'Set pos: ' + str(self.cur_x) + ' : ' + str(self.cur_y) + ' : ' + str(int(self.cur_psi * (180 / math.pi))) )

    def set_control(self, left, right):
        if left < -20:
            left = -20
        if left > 20:
            left = 20
        if right < -20:
            right = -20
        if right > 20:
            right = 20
        self.sock.sendall(pickle.dumps([int(left), int(right)]))
        if (self.isLog):
            print(self.teg + 'Socket send: ' + str(int(left)) + ' : ' + str(int(right)))
            
    def get_id(self):
        return  self.ID

    def follow_path(self, coordinates):
        if (self.isLog):
            labeled_coords = [f"Point {i + 1}: {coord}" for i, coord in enumerate(coordinates)]
            coord_str = ', '.join(labeled_coords)
            print(self.teg + 'Coord: ' + coord_str)
        