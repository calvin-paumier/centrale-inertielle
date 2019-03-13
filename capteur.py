#!/usr/bin/env python3

import smbus
import time
import os
import math

def c2(val, bits):
    if (val & (1 << (bits - 1))) != 0:
        val = val - (1 << bits)
    return val

class Capteur():
    bus = smbus.SMBus(1)

    # offsets gyroscope
    x_offset = 0
    y_offset = 0
    z_offset = 0

    # angle
    x_angle = 0
    y_angle = 0
    x_gyro_angle = 0
    y_gyro_angle = 0
    temps = 0
    test = 0

    # constante
    gyro_sensibility = 250.0
    accel_sensibility = 8.0
    precision = 1<<15
    K=0.95 # Constante du filtre
    

    # Initialisation du capteur
    def __init__(self):
        self.enableAccel() # Activation accelerometre
        self.enableGyro() # Activation gyroscope
        self.enableMagnet() # Activation magnetometre
        self.enableFilter() # Activation filtre passe-bas

    # Accelerometre
    def enableAccel(self):
        self.bus.write_byte_data(0x68, 0x1c, self.bus.read_byte_data(0x68, 0x1c) | 0x10) # Reglage de la sensibilite de l'accelerometre

    def getAccelData(self):
        data = self.bus.read_i2c_block_data(0x68, 0x3b, 6)
        x = self.accel_sensibility / self.precision * c2(data[0] << 8 | data[1], 16)
        y = self.accel_sensibility / self.precision * c2(data[2] << 8 | data[3], 16)
        z = self.accel_sensibility / self.precision * c2(data[4] << 8 | data[5], 16)
        return (x,y,z)

    def getAccelAngle(self):
        (x,y,z) = self.getAccelData()
        l = math.sqrt(x*x + y*y + z*z)
        return (math.degrees(math.asin(y/l)), -math.degrees(math.asin(x/l)))

    # Temperature
    def getTemp(self):
        data = self.bus.read_i2c_block_data(0x68, 0x41, 2)
        Temp = (c2(data[0] << 8 | data[1],16)*125/65536)+21
        return Temp

    # Gyroscope
    def enableGyro(self):
        #self.bus.write_byte_data(0x68, 0x1b, self.bus.read_byte_data(0x68, 0x1b) | 0x08) # Reglage de la sensibilite du gyroscope
        for i in range (100):
            data = self.bus.read_i2c_block_data(0x68, 0x43, 6)
            self.x_offset -= self.gyro_sensibility / self.precision * c2(data[0] << 8 | data[1], 16) / 100
            self.y_offset -= self.gyro_sensibility / self.precision * c2(data[2] << 8 | data[3], 16) / 100
            self.z_offset -= self.gyro_sensibility / self.precision * c2(data[4] << 8 | data[5], 16) / 100
            time.sleep(0.015)

    def getGyroData(self):
        data = self.bus.read_i2c_block_data(0x68, 0x43, 6)
        x = self.gyro_sensibility / self.precision * c2(data[0] << 8 | data[1], 16) + self.x_offset
        y = self.gyro_sensibility / self.precision * c2(data[2] << 8 | data[3], 16) + self.y_offset
        z = self.gyro_sensibility / self.precision * c2(data[4] << 8 | data[5], 16) + self.z_offset
        return (x,y,z)

    def getGyroAngle(self):
        (x, y, z) = self.getGyroData()
        (x_acc_angle, y_acc_angle) = self.getAccelAngle()
        t = self.temps
        self.temps = time.time()
        if t == 0:  # Initialisation sur plan incline
            self.x_gyro_angle = x_acc_angle
            self.y_gyro_angle = y_acc_angle
        else:
            t = self.temps - t
            #print(t)
            self.x_gyro_angle += x * t
            self.y_gyro_angle += y * t
            x = self.x_gyro_angle
            y = self.y_gyro_angle
            self.x_gyro_angle += y * math.sin(math.radians(z * t))
            self.y_gyro_angle -= x * math.sin(math.radians(z * t))
        return (self.x_gyro_angle, self.y_gyro_angle)

    # Angle
    def getAngle(self): #Filtre complementaire
#        (x_acc_angle, y_acc_angle) = self.getAccelAngle()
#        (x, y, z) = self.getGyroData()
#        if (self.test == 0):
#            self.x_angle = x_acc_angle
#            self.y_angle = y_acc_angle
#            self.test=1
#        else:
#            self.x_angle = self.K * self.x_angle + self.K * x * 0.0125 + self.K * x_acc_angle
#            self.y_angle = self.K * self.y_angle + self.K * y * 0.0125 + self.K * y_acc_angle
#        return(self.x_angle, self.y_angle)
        (x_acc_angle, y_acc_angle) = self.getAccelAngle()
        (x_g_angle, y_g_angle) = self.getGyroAngle()
        self.x_angle = self.K * x_g_angle + (1 - self.K) * x_acc_angle
        self.y_angle = self.K * y_g_angle + (1 - self.K) * y_acc_angle
        return(self.x_angle, self.y_angle)
        
    # Magnetometre
    def enableMagnet(self):
        self.bus.write_byte_data(0x68, 0x37, 0x02)
        self.bus.write_byte_data(0x0c, 0x0a, 0x00) # Mode Power down du magnetometre
        self.bus.write_byte_data(0x0c, 0x0a, 0x01<<4 | 0x06) # Reglage du magnetometre

    def getMagnetReady(self):
        return self.bus.read_byte_data(0x0c, 0x02) & 0x01

    def getMagnetData(self):
        data = self.bus.read_i2c_block_data(0x0c, 0x03, 7)
        s = data[6]
        x = 0
        y = 0
        z = 0
        if (s & 0x08 == 0):
            x = c2(data[1] << 8 | data[0],16)*0.0015
            y = c2(data[3] << 8 | data[2],16)*0.0015
            z = c2(data[5] << 8 | data[4],16)*0.0015
        return (s,x,y,z)

    # Filtre passe-bas
    def enableFilter(self):
        self.bus.write_byte_data(0x68, 0x1a, 0x03)

def printAccel():
    (x, y, z) = capteur.getAccelData()
    (x_acc_angle, y_acc_angle) = capteur.getAccelAngle()
    print("Accel  X=%7.4f Y=%7.4f Z=%7.4f A=%7.4f/%7.4f" % (x, y, z, x_acc_angle, y_acc_angle))

def printTemp():
    t = capteur.getTemp()
    print("Temp T=%7.4f" % (t))

def printGyro():
    (x, y, z) = capteur.getGyroData()
    (x_gyro_angle, y_gyro_angle) = capteur.getGyroAngle()
    print("Gyro   X=%7.4f Y=%7.4f Z=%7.4f A=%7.4f/%7.4f" % (x, y, z, x_gyro_angle, y_gyro_angle))

def printMagnet():
    if capteur.getMagnetReady():
        (s,x,y,z) = capteur.getMagnetData()
        print("Magnet S=%7.4f, X=%7.4f Y=%7.4f Z=%7.4f" % (s, x, y, z))
        
def printAngle(): 
    (xa, ya) = capteur.getAngle()
    print("Roll = %7.4f, Pitch = %7.4f" %(ya, xa))

capteur = Capteur()


while (1):
    time.sleep(0.01)
    for i in range(100):
        capteur.getGyroAngle()
        time.sleep(0.01)
    #printAccel()
    #printTemp()
    #printMagnet()
    #printGyro()
    printAngle()
