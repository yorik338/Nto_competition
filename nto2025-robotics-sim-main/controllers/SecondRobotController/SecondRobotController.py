"""SecondRobotController controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import socket
import threading
import pickle

def handler_callback(conn: socket.socket, data):
    while True:
        msg = conn.recv(1024)
        info = pickle.loads(msg)
        data[0] = info[0]
        data[1] = info[1]

HOST = "127.0.0.1"
PORT = 27002
list_of_data = [0, 0]

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((HOST, PORT))
sock.listen()

# Wait for connection
conn, addr = sock.accept()
thr = threading.Thread(target=handler_callback, args=(conn, list_of_data))
thr.start()

# create the Robot instance.
robot = Robot()
leftMotor = robot.getDevice('LeftMotor')
rightMotor = robot.getDevice('RightMotor')

leftMotor.setVelocity(0)
rightMotor.setVelocity(0)
leftMotor.setPosition(float('+inf'))
rightMotor.setPosition(float('+inf'))

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    wl = list_of_data[0]
    wr = list_of_data[1]

    leftMotor.setVelocity(wl)
    rightMotor.setVelocity(wr)
