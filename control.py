import sim as vrep
import time
import numpy as np
#predef
class Bot:

   #conecta na instanciacao do objeto
    def __init__(self):
        self.sensors = []
        self.position = []
        self.orientation = []
        self.detection = []
        vrep.simxFinish(-1)
        self.ID = vrep.simxStart('127.0.0.1', 22222, True, True, 5000, 5)
        if self.ID != -1:
            print("Conectado ao servidor, ID = "+ str(self.ID))
        else:
            print("Falha ao conectar")
            sys.exit()
#processa os handlers
        e, self.handler = vrep.simxGetObjectHandle(self.ID, "dr20", vrep.simx_opmode_blocking)
        e, self.Lmotor = vrep.simxGetObjectHandle(self.ID, "dr20_leftWheelJoint_", vrep.simx_opmode_blocking)
        e, self.Rmotor = vrep.simxGetObjectHandle(self.ID, "dr20_rightWheelJoint_", vrep.simx_opmode_blocking)

        ret, sensors_L = vrep.simxGetObjectHandle(self.ID, "dr20_infraredSensor5_",vrep.simx_opmode_blocking)
        self.sensors.append(sensors_L)

        ret, sensors_FL = vrep.simxGetObjectHandle(self.ID, "dr20_infraredSensor6_",vrep.simx_opmode_blocking)
        self.sensors.append(sensors_FL)

        ret, ultra = vrep.simxGetObjectHandle(self.ID, "dr20_ultrasonicSensor_", vrep.simx_opmode_blocking)
        self.sensors.append(ultra)

        ret, sensors_FR = vrep.simxGetObjectHandle(self.ID, "dr20_infraredSensor1_",vrep.simx_opmode_blocking)
        self.sensors.append(sensors_FR)
        ret, sensors_R = vrep.simxGetObjectHandle(self.ID,"dr20_infraredSensor2_",vrep.simx_opmode_blocking)
        self.sensors.append(sensors_R)

        e, self.position = vrep.simxGetObjectPosition(self.ID, self.handler, -1, vrep.simx_opmode_streaming)
        e, self.orientation = vrep.simxGetObjectOrientation(self.ID, self.handler, -1, vrep.simx_opmode_streaming)

        #scan inicial
        for sensor in self.sensors:
            vrep.simxReadProximitySensor(self.ID, sensor, vrep.simx_opmode_streaming)

#TODO: refazer essa logica
    def move(self, target):
        if target is None:
            vrep.simxSetJointTargetVelocity(self.ID, self.Lmotor, 5, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(self.ID, self.Rmotor, 5, vrep.simx_opmode_streaming)
        else:
            target.append(0.0)
            curfacing = self.orientation
            dest = np.asarray(target)
            print ("moving to "+str(dest))
            ihat= np.asarray(curfacing)/np.linalg.norm(curfacing)
            jhat = np.asarray(dest)/np.linalg.norm(dest)

            dot_product = np.dot(ihat,jhat)
            angle = int(np.degrees(np.arccos(dot_product)))
            print(str(np.trunc(angle)))
            self.turn(np.trunc(angle))
            self.stop()
            vrep.simxSetJointTargetVelocity(self.ID, self.Lmotor, 5, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetVelocity(self.ID, self.Rmotor, 5, vrep.simx_opmode_oneshot)

            while self.position[1] == dest[1] and self.position[0] == dest[0]:
                print("moving")

            vrep.simxSetJointTargetVelocity(self.ID, self.Lmotor, 0, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(self.ID, self.Rmotor, 0, vrep.simx_opmode_streaming)


#       algebra pra virar até o ponto certo: aplicar um vetor de rotaçao pra saber a orientacao desejada, usar a funcao turn pra girar
#
    def stop(self):
        vrep.simxSetJointTargetVelocity(self.ID, self.Lmotor, 0, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(self.ID, self.Rmotor, 0, vrep.simx_opmode_streaming)
#
    def turn(self, target):
        #normaliza angulo
        target = np.array([0.0,0.0, float(target)])
        target = np.degrees(np.unwrap(target))

        dest = np.around((target),2)

        if ((target.flat[2]-self.orientation.flat[2] + 360)%360<180):
            #gira anti-horario
            print("anti")
            vrep.simxSetJointTargetVelocity(self.ID, self.Lmotor, 0, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(self.ID, self.Rmotor, 2, vrep.simx_opmode_streaming)

        else:#gira horario
            vrep.simxSetJointTargetVelocity(self.ID, self.Lmotor, 2, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(self.ID, self.Rmotor, 0, vrep.simx_opmode_streaming)
        while True:
            print("currently facing " + str(self.orientation[2])+ ", target: "+str(dest[2]))
            self.facing()
            if (np.trunc(self.orientation[2]) == np.trunc(dest[2])):
                print ("---------  AE CARAIO -------------")
                break

    def facing(self):
        e, face  = vrep.simxGetObjectOrientation(self.ID, self.handler, -1, vrep.simx_opmode_buffer)
        self.orientation = np.around(np.degrees(np.array(face)),2)

        return self.orientation

    def update_position(self):
        e, pos = vrep.simxGetObjectPosition(self.ID, self.handler, -1, vrep.simx_opmode_buffer)
        self.position = np.asarray(pos)
        return self.position
    def scan(self):
        buf = []
        for sensor in self.sensors:
            ret, detected, detected_point, detected_handle, detected_normalvec = vrep.simxReadProximitySensor(self.ID, sensor, vrep.simx_opmode_buffer)
            buf.append(detected_point)
        a = np.asarray(buf)
        self.seeing = a
        return self.seeing
#class World:
#	size
#	meshsize
#
#
robot = Bot()
print(vrep.simxStartSimulation(robot.ID, vrep.simx_opmode_oneshot))
while True:
    time.sleep(3)
    robot.update_position()
    robot.facing()
    robot.scan()
    time.sleep(2)
    print ("@ "+str(robot.position)+"\t"+str(robot.orientation)+"\t"+str(robot.seeing))
    time.sleep(2)
    robot.move([-1.0,1.0])
    time.sleep(5)
