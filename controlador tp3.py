from controller import Robot, GPS, DistanceSensor

robot = Robot()
timeStep = int(robot.getBasicTimeStep())

wheel_left = robot.getDevice("wheel1 motor")
wheel_right = robot.getDevice("wheel2 motor")
wheel_left.setPosition(float('inf'))
wheel_right.setPosition(float('inf'))

gps = robot.getDevice("gps")
gps.enable(timeStep)

ds_front = robot.getDevice("distance sensor1")
ds_front.enable(timeStep)

ds_back = robot.getDevice("distance sensor2")
ds_back.enable(timeStep)

vel_avance = 6.28
vel_retroceso = -6.28

estado = "avanzar"
cnt_pasos = 0
pasos_por_baldosa = 0
baldosa_inicial = None

baldosa_inicial_coords = None
distancia_tolerancia = 0.1  

while robot.step(timeStep) != -1:
    pos = gps.getValues()
    distancia_frontal = ds_front.getValue()
    distancia_trasera = ds_back.getValue()

    if baldosa_inicial is None:
        baldosa_inicial = pos[0], pos[2]

    if baldosa_inicial_coords is None:
        baldosa_inicial_coords = pos[0], pos[2]

    if estado == "avanzar":
        if distancia_frontal > 0.2:  
            wheel_left.setVelocity(vel_avance)
            wheel_right.setVelocity(vel_avance)
        else:
            wheel_left.setVelocity(0)
            wheel_right.setVelocity(0)
            estado = "retroceder"

    elif estado == "retroceder":
        if distancia_trasera > 0.2:  
            wheel_left.setVelocity(vel_retroceso)
            wheel_right.setVelocity(vel_retroceso)
        else:
            wheel_left.setVelocity(0)
            wheel_right.setVelocity(0)
            estado = "avanzar"
            cnt_pasos += 1
            if (abs(pos[0] - baldosa_inicial_coords[0]) < distancia_tolerancia and
                abs(pos[2] - baldosa_inicial_coords[1]) < distancia_tolerancia):
                pasos_por_baldosa += 1

    if pasos_por_baldosa >= 3:
        wheel_left.setVelocity(0)
        wheel_right.setVelocity(0)
        break

wheel_left.setVelocity(0)
wheel_right.setVelocity(0)
