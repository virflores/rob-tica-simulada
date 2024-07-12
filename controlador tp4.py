from controller import GPS, DistanceSensor, Robot, Giroscopio
import time
import random

class RobotController:
    def __init__(self, robot):
        self.robot = Robot
        self.timestep = int(robot.getBasicTimeStep())
        
        # Iniciar sensores de distancia
        self.front_distance_sensor = robot.getDevice("front_distance_sensor")
        self.front_distance_sensor.enable(self.timestep)
        
        self.left_distance_sensor = robot.getDevice("left_distance_sensor")
        self.left_distance_sensor.enable(self.timestep)
        
        self.right_distance_sensor = robot.getDevice("right_distance_sensor")
        self.right_distance_sensor.enable(self.timestep)
        
        # Iniciar GPS
        self.gps = robot.getDevice("gps")
        self.gps.enable(self.timestep)
        
        # Iniciar Giroscopio
        self.gyro = robot.getDevice("gyro")
        self.gyro.enable(self.timestep)
        
        # Iniciar Encoders
        self.left_encoder = robot.getDevice("left_encoder")
        self.right_encoder = robot.getDevice("right_encoder")
        self.left_encoder.enable(self.timestep)
        self.right_encoder.enable(self.timestep)
        
        # Iniciar motores
        self.left_motor = robot.getDevice("left_motor")
        self.right_motor = robot.getDevice("right_motor")
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        
        # Estado inicial
        self.state = "AVANZAR"

    def read_sensors(self):
        self.front_distance = self.front_distance_sensor.getValue()
        self.left_distance = self.left_distance_sensor.getValue()
        self.right_distance = self.right_distance_sensor.getValue
