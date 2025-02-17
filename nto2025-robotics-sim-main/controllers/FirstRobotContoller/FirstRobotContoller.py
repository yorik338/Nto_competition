from controller import Robot, Motor, Keyboard
import socket
import threading
import pickle


def handler_callback(conn: socket.socket, data):
    while True:
        msg = conn.recv(1024)
        if not msg:
            break
        info = pickle.loads(msg)
        data[0] = info[0]
        data[1] = info[1]


HOST = "127.0.0.1"
PORT = 27001
list_of_data = [0, 0]

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((HOST, PORT))
sock.listen()

# Ожидание соединения
conn, addr = sock.accept()
thr = threading.Thread(target=handler_callback, args=(conn, list_of_data))
thr.start()

# Создаем экземпляр робота и получаем моторы
robot = Robot()
leftMotor = robot.getDevice('LeftMotor')
rightMotor = robot.getDevice('RightMotor')

leftMotor.setVelocity(0)
rightMotor.setVelocity(0)
leftMotor.setPosition(float('+inf'))
rightMotor.setPosition(float('+inf'))

timestep = int(robot.getBasicTimeStep())

# Основной цикл симуляции
while robot.step(timestep) != -1:
    wl = list_of_data[0]
    wr = list_of_data[1]

    leftMotor.setVelocity(wl)
    rightMotor.setVelocity(wr)
