class AdvancedRobotController:
    def __init__(self, robot):
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())
        
        self.front_distance_sensor = robot.getDevice("front_distance_sensor")
        self.front_distance_sensor.enable(self.timestep)
        
        self.left_distance_sensor = robot.getDevice("left_distance_sensor")
        self.left_distance_sensor.enable(self.timestep)
        
        self.right_distance_sensor = robot.getDevice("right_distance_sensor")
        self.right_distance_sensor.enable(self.timestep)
        
        self.gps = robot.getDevice("gps")
        self.gps.enable(self.timestep)
        
        self.gyro = robot.getDevice("gyro")
        self.gyro.enable(self.timestep)
        
        self.left_encoder = robot.getDevice("left_encoder")
        self.right_encoder = robot.getDevice("right_encoder")
        self.left_encoder.enable(self.timestep)
        self.right_encoder.enable(self.timestep)
        
        self.imu = robot.getDevice("imu")
        self.imu.enable(self.timestep)
        
        self.color_sensor = robot.getDevice("color_sensor")
        self.color_sensor.enable(self.timestep)
        
        self.left_motor = robot.getDevice("left_motor")
        self.right_motor = robot.getDevice("right_motor")
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
    
        self.state = "AVANZAR"

    def read_sensors(self):
        self.front_distance = self.front_distance_sensor.getValue()
        self.left_distance = self.left_distance_sensor.getValue()
        self.right_distance = self.right_distance_sensor.getValue()
        self.position = self.gps.getValues()
        self.orientation = self.gyro.getValues()
        self.imu_data = self.imu.getRollPitchYaw()
        self.left_pulses = self.left_encoder.getValue()
        self.right_pulses = self.right_encoder.getValue()
        self.color = self.color_sensor.getValue()

    def print_telemetry(self):
        print(f"Distancia Frontal: {self.front_distance}")
        print(f"Distancia Izquierda: {self.left_distance}")
        print(f"Distancia Derecha: {self.right_distance}")
        print(f"Posición (X, Y, Z): {self.position}")
        print(f"Orientación (grados): {self.orientation}")
        print(f"IMU - Roll: {self.imu_data[0]}, Pitch: {self.imu_data[1]}, Yaw: {self.imu_data[2]}")
        print(f"Pulsos Encoder Izquierdo: {self.left_pulses}")
        print(f"Pulsos Encoder Derecho: {self.right_pulses}")
        print(f"Color Detectado: {self.color}")
        print(f"Estado: {self.state}")
        
    def navigate(self):
        if self.state == "AVANZAR":
            if self.front_distance < 0.5 or self.color == "rojo":  
                self.state = "GIRAR"
            else:
                self.left_motor.setVelocity(5.0)
                self.right_motor.setVelocity(5.0)
        elif self.state == "GIRAR":
            if self.orientation[2] < 90:  
                self.left_motor.setVelocity(2.0)
                self.right_motor.setVelocity(-2.0)
            else:
                self.state = "AVANZAR"
                
    def run(self):
        while self.robot.step(self.timestep) != -1:
            self.read_sensors()
            self.print_telemetry()
            self.navigate()

robot = Robot()  
controller = AdvancedRobotController(robot)
controller.run()
