# -*- coding: utf-8 -*-
#/***************************************************************************************************************
#* Razor AHRS Firmware v1.4.2
#* 9 Degree of Measurement Attitude and Heading Reference System
#* for Sparkfun "9DOF Razor IMU" (SEN-10125 and SEN-10736)
#* and "9DOF Sensor Stick" (SEN-10183, 10321 and SEN-10724)
#*
#* Released under GNU GPL (General Public License) v3.0
#* Copyright (C) 2013 Peter Bartz [http://ptrbrtz.net]
#* Copyright (C) 2011-2012 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
#*
#* Infos, updates, bug reports, contributions and feedback:
#*     https://github.com/ptrbrtz/razor-9dof-ahrs
#*
#*
#* History:
#*   * Original code (http://code.google.com/p/sf9domahrs/) by Doug Weibel and Jose Julio,
#*     based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel. Thank you!
#*
#*   * Updated code (http://groups.google.com/group/sf_9dof_ahrs_update) by David Malik (david.zsolt.malik@gmail.com)
#*     for new Sparkfun 9DOF Razor hardware (SEN-10125).
#*
#*   * Updated and extended by Peter Bartz (peter-bartz@gmx.de):
#*     * v1.3.0
#*       * Cleaned up, streamlined and restructured most of the code to make it more comprehensible.
#*       * Added sensor calibration (improves precision and responsiveness a lot!).
#*       * Added binary yaw/pitch/roll output.
#*       * Added basic serial command interface to set output modes/calibrate sensors/synch stream/etc.
#*       * Added support to synch automatically when using Rovering Networks Bluetooth modules (and compatible).
#*       * Wrote new easier to use test program (using Processing).
#*       * Added support for new version of "9DOF Razor IMU": SEN-10736.
#*       --> The output of this code is not compatible with the older versions!
#*       --> A Processing sketch to test the tracker is available.
#*     * v1.3.1
#*       * Initializing rotation matrix based on start-up sensor readings -> orientation OK right away.
#*       * Adjusted gyro low-pass filter and output rate settings.
#*     * v1.3.2
#*       * Adapted code to work with new Arduino 1.0 (and older versions still).
#*     * v1.3.3
#*       * Improved synching.
#*     * v1.4.0
#*       * Added support for SparkFun "9DOF Sensor Stick" (versions SEN-10183, SEN-10321 and SEN-10724).
#*     * v1.4.1
#*       * Added output modes to read raw and/or calibrated sensor data in text or binary format.
#*       * Added static magnetometer soft iron distortion compensation
#*     * v1.4.2
#*       * (No core firmware changes)
#*
#* TODOs:
#*   * Allow optional use of EEPROM for storing and reading calibration values.
#*   * Use self-test and temperature-compensation features of the sensors.
#***************************************************************************************************************/

import math
import ast
import numpy
from scipy import linalg
import pylab
try:
    from .config import preferences
    from . import kalman
except:
    from config import preferences
    import kalman


class DCM(object):

    def __init__(self):
        self.prefs = preferences.Prefs()
        self.sensorFusionSet = False
        self.magnetomCal = False
        self.G_Dt = 0
        self.accel = []
        self.accel_min = []
        self.tempAccel_min = []
        self.accel_max = []
        self.tempAccel_max = []

        self.magnetom = []
        self.magnetom_min = []
        self.magnetom_max = []
        self.magnetom_tmp = []

        self.gyro = []
        self.gyro_average = []
        self.gyro_num_samples = 0

        self.MAG_Heading = 0
        self.Accel_Vector = [0, 0, 0]  # Store the acceleration in a vector
        self.Gyro_Vector = [0, 0, 0]  # Store the gyros turn rate in a vector
        self.Omega_Vector = [0, 0, 0]  # Corrected Gyro_Vector data
        self.Omega_P = [0, 0, 0]  # Omega Proportional correction
        self.Omega_I = [0, 0, 0]  # Omega Integrator
        self.Omega = [0, 0, 0]
        self.errorRollPitch = [0, 0, 0]
        self.errorYaw = [0, 0, 0]
        self.DCM_Matrix = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        self.Update_Matrix = [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
        self.Temporary_Matrix = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

        #Kalman Filter
        self.A = numpy.matrix([1])
        self.H = numpy.matrix([1])
        self.B = numpy.matrix([0])
        self.Q = numpy.matrix([0.01])
        self.R = numpy.matrix([0.1])
        self.xhat = numpy.matrix([3])
        self.P = numpy.matrix([1])
        self.kfilterAccelX = kalman.KalmanFilterLinear(self.A, self.B, self.H, self.xhat, self.P, self.Q, self.R)
        self.kfilterAccelY = kalman.KalmanFilterLinear(self.A, self.B, self.H, self.xhat, self.P, self.Q, self.R)
        self.kfilterAccelZ = kalman.KalmanFilterLinear(self.A, self.B, self.H, self.xhat, self.P, self.Q, self.R)
        self.measuredAccelX = []
        self.measuredAccelY = []
        self.measuredAccelZ = []
        self.kalmanAccelX = []
        self.kalmanAccelY = []
        self.kalmanAccelZ = []
        self.kfilterMagnetomX = kalman.KalmanFilterLinear(self.A, self.B, self.H, self.xhat, self.P, self.Q, self.R)
        self.kfilterMagnetomY = kalman.KalmanFilterLinear(self.A, self.B, self.H, self.xhat, self.P, self.Q, self.R)
        self.kfilterMagnetomZ = kalman.KalmanFilterLinear(self.A, self.B, self.H, self.xhat, self.P, self.Q, self.R)
        self.measuredMagnetomX = []
        self.measuredMagnetomY = []
        self.measuredMagnetomZ = []
        self.kalmanMagnetomX = []
        self.kalmanMagnetomY = []
        self.kalmanMagnetomZ = []
        self.kfilterGyroX = kalman.KalmanFilterLinear(self.A, self.B, self.H, self.xhat, self.P, self.Q, self.R)
        self.kfilterGyroY = kalman.KalmanFilterLinear(self.A, self.B, self.H, self.xhat, self.P, self.Q, self.R)
        self.kfilterGyroZ = kalman.KalmanFilterLinear(self.A, self.B, self.H, self.xhat, self.P, self.Q, self.R)
        self.measuredGyroX = []
        self.measuredGyroY = []
        self.measuredGyroZ = []
        self.kalmanGyroX = []
        self.kalmanGyroY = []
        self.kalmanGyroZ = []

        # Euler angles
        self.yaw = 0
        self.pitch = 0
        self.roll = 0

        #Position and Velocity
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]

        self.GRAVITY = float(self.prefs.getSensor("GRAVITY"))  # "1G reference" used for DCM filter and accelerometer calibration
        self.tempGRAVITY = []
        # DCM parameters
        self.Kp_ROLLPITCH = float(self.prefs.getSensor("Kp_ROLLPITCH"))
        self.Ki_ROLLPITCH = float(self.prefs.getSensor("Ki_ROLLPITCH"))
        self.Kp_YAW = float(self.prefs.getSensor("Kp_YAW"))
        self.Ki_YAW = float(self.prefs.getSensor("Ki_YAW"))

        #####################################Calibration####################################
        # Accelerometer
        # "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
        """self.prefs.setSensor("ACCEL_X_MIN", 0)
        self.prefs.setSensor("ACCEL_X_MAX", 0)
        self.prefs.setSensor("ACCEL_Y_MIN", 0)
        self.prefs.setSensor("ACCEL_Y_MAX", 0)
        self.prefs.setSensor("ACCEL_Z_MIN", 0)
        self.prefs.setSensor("ACCEL_Z_MAX", 0)"""

        self.ACCEL_X_MIN = float(self.prefs.getSensor("ACCEL_X_MIN"))
        self.ACCEL_X_MAX = float(self.prefs.getSensor("ACCEL_X_MAX"))
        self.ACCEL_Y_MIN = float(self.prefs.getSensor("ACCEL_Y_MIN"))
        self.ACCEL_Y_MAX = float(self.prefs.getSensor("ACCEL_Y_MAX"))
        self.ACCEL_Z_MIN = float(self.prefs.getSensor("ACCEL_Z_MIN"))
        self.ACCEL_Z_MAX = float(self.prefs.getSensor("ACCEL_Z_MAX"))

        # Magnetometer (standard calibration mode)
        # "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
        self.MAGN_X_MIN = float(self.prefs.getSensor("MAGN_X_MIN"))
        self.MAGN_X_MAX = float(self.prefs.getSensor("MAGN_X_MAX"))
        self.MAGN_Y_MIN = float(self.prefs.getSensor("MAGN_Y_MIN"))
        self.MAGN_Y_MAX = float(self.prefs.getSensor("MAGN_Y_MAX"))
        self.MAGN_Z_MIN = float(self.prefs.getSensor("MAGN_Z_MIN"))
        self.MAGN_Z_MAX = float(self.prefs.getSensor("MAGN_Z_MAX"))

        # Magnetometer (extended calibration mode)
        # Uncommend to use extended magnetometer calibration (compensates hard & soft iron errors)
        self.magn_ellipsoid_center = ast.literal_eval(self.prefs.getSensor("magn_ellipsoid_center"))
        self.magn_ellipsoid_transform = ast.literal_eval(self.prefs.getSensor("magn_ellipsoid_transform"))

        # Gyroscope
        # "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
        self.GYRO_AVERAGE_OFFSET_X = float(self.prefs.getSensor("GYRO_AVERAGE_OFFSET_X"))
        self.GYRO_AVERAGE_OFFSET_Y = float(self.prefs.getSensor("GYRO_AVERAGE_OFFSET_Y"))
        self.GYRO_AVERAGE_OFFSET_Z = float(self.prefs.getSensor("GYRO_AVERAGE_OFFSET_Z"))

        self.GYRO_GAIN = float(self.prefs.getSensor("GYRO_GAIN"))  # Same gain on all axes

        ##################################################################################
        """self.prefs.setSensor("ACCEL_X_OFFSET", 0)
        self.prefs.setSensor("ACCEL_Y_OFFSET", 0)
        self.prefs.setSensor("ACCEL_Z_OFFSET", 0)
        self.prefs.setSensor("ACCEL_X_SCALE", 1)
        self.prefs.setSensor("ACCEL_Y_SCALE", 1)
        self.prefs.setSensor("ACCEL_Z_SCALE", 1)"""

        self.ACCEL_X_OFFSET = float(self.prefs.getSensor("ACCEL_X_OFFSET"))
        self.ACCEL_Y_OFFSET = float(self.prefs.getSensor("ACCEL_Y_OFFSET"))
        self.ACCEL_Z_OFFSET = float(self.prefs.getSensor("ACCEL_Z_OFFSET"))
        self.ACCEL_X_SCALE = float(self.prefs.getSensor("ACCEL_X_SCALE"))
        self.ACCEL_Y_SCALE = float(self.prefs.getSensor("ACCEL_Y_SCALE"))
        self.ACCEL_Z_SCALE = float(self.prefs.getSensor("ACCEL_Z_SCALE"))

        self.MAGN_X_OFFSET = float(self.prefs.getSensor("MAGN_X_OFFSET"))
        self.MAGN_Y_OFFSET = float(self.prefs.getSensor("MAGN_Y_OFFSET"))
        self.MAGN_Z_OFFSET = float(self.prefs.getSensor("MAGN_Z_OFFSET"))
        self.MAGN_X_SCALE = float(self.prefs.getSensor("MAGN_X_SCALE"))
        self.MAGN_Y_SCALE = float(self.prefs.getSensor("MAGN_Y_SCALE"))
        self.MAGN_Z_SCALE = float(self.prefs.getSensor("MAGN_Z_SCALE"))

    def resetSensorFusion(self):
        self.sensorFusionSet = True
        temp1 = []
        temp2 = []
        xAxis = [1.0, 0.0, 0.0]

        #Reset Vectors in case of error
        self.Omega_Vector = [0, 0, 0]  # Corrected Gyro_Vector data
        self.Omega_P = [0, 0, 0]  # Omega Proportional correction
        self.Omega_I = [0, 0, 0]  # Omega Integrator
        self.Omega = [0, 0, 0]
        self.errorRollPitch = [0, 0, 0]
        self.errorYaw = [0, 0, 0]
        self.DCM_Matrix = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        self.Update_Matrix = [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
        self.Temporary_Matrix = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

        # GET PITCH
        # Using y-z-plane-component/x-component of gravity vector
        # raw_input(self.accel)
        self.pitch = -math.atan2(self.accel[0], math.sqrt(self.accel[1] * self.accel[1] + self.accel[2] * self.accel[2]))

        # GET ROLL
        # Compensate pitch of gravity vector
        temp1 = self.vectorCrossProduct(self.accel, xAxis)
        temp2 = self.vectorCrossProduct(xAxis, temp1)
        # Normally using x-z-plane-component/y-component of compensated gravity vector
        # roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
        # Since we compensated for pitch, x-z-plane-component equals z-component:
        self.roll = math.atan2(temp2[1], temp2[2])

        # GET YAW
        self.compassHeading()
        self.yaw = self.MAG_Heading

        # Init rotation matrix
        self.DCM_Matrix = self.initRotationMatrix(self.yaw, self.pitch, self.roll)

    def setSensorData(self, data):
        self.accel = []
        self.magnetom = []
        self.gyro = []
        for key, val in list(data.items()):
            if "G_Dt" in key:
                self.G_Dt = float(data["G_Dt"])
            if "Accel" in key:
                accel = data["Accel"]
                for i in accel:
                    self.accel.append(float(i))
            if "Magnetom" in key:
                magnetom = data["Magnetom"]
                for i in magnetom:
                    self.magnetom.append(float(i))
            if "Gyro" in key:
                gyro = data["Gyro"]
                for i in gyro:
                    self.gyro.append(float(i))
        if self.sensorFusionSet is False:
            self.resetSensorFusion()
        else:
            self.compensateSensorErrors()
            self.getVelocity()
            self.compassHeading()
            self.matrixUpdate()
            self.normalize()
            self.driftCorrection()
            self.eulerAngles()
        if self.yaw < 0:
            self.yaw = math.pi + (math.pi + self.yaw)
        if self.pitch < 0:
            self.pitch = math.pi + (math.pi + self.pitch)
        if self.roll < 0:
            self.roll = math.pi + (math.pi + self.roll)

        data = [self.yaw, self.pitch, self.roll]
        # print((data))
        return data

    # Apply calibration to raw sensor readings
    def compensateSensorErrors(self):
        # Compensate accelerometer error
        temp = []
        #self.measuredAccelX.append(self.accel[0])
        accelX = self.kfilterAccelX.GetCurrentState()[0, 0]
        self.kalmanAccelX.append(accelX)
        self.kfilterAccelX.Step(numpy.matrix([0]), numpy.matrix([self.accel[0]]))
        temp.append((accelX - self.ACCEL_X_OFFSET) * self.ACCEL_X_SCALE)

        #self.measuredAccelY.append(self.accel[1])
        accelY = self.kfilterAccelY.GetCurrentState()[0, 0]
        self.kalmanAccelY.append(accelY)
        self.kfilterAccelY.Step(numpy.matrix([0]), numpy.matrix([self.accel[1]]))
        temp.append((accelY - self.ACCEL_Y_OFFSET) * self.ACCEL_Y_SCALE)

        #self.measuredAccelZ.append(self.accel[2])
        accelZ = self.kfilterAccelZ.GetCurrentState()[0, 0]
        self.kalmanAccelZ.append(accelZ)
        self.kfilterAccelZ.Step(numpy.matrix([0]), numpy.matrix([self.accel[2]]))
        temp.append((accelZ - self.ACCEL_Z_OFFSET) * self.ACCEL_Z_SCALE)

        self.accel = temp

        avg = 0
        minX = 0
        maxX = 0
        offset = 0
        scale = 0
        if len(self.kalmanAccelX) is 200:
            for i in self.kalmanAccelX:
                avg += i
            avg = avg / len(self.kalmanAccelX)
            if avg > 240.0:
                maxX = avg
            if maxX > self.ACCEL_X_MAX:
                self.prefs.setSensor("ACCEL_X_MAX", maxX)
                self.ACCEL_X_MAX = maxX
                print((self.ACCEL_X_MAX))
                offset = (self.ACCEL_X_MIN + self.ACCEL_X_MAX) / 2.0
                self.prefs.setSensor("ACCEL_X_OFFSET", offset)
                self.ACCEL_X_OFFSET = offset
                scale = self.GRAVITY / (self.ACCEL_X_MAX - self.ACCEL_X_OFFSET)
                self.prefs.setSensor("ACCEL_X_SCALE", scale)
                self.ACCEL_X_SCALE = scale
            if avg < -240.0:
                minX = avg
            if minX < self.ACCEL_X_MIN:
                self.prefs.setSensor("ACCEL_X_MIN", minX)
                self.ACCEL_X_MIN = minX
                print((self.ACCEL_X_MIN))
                offset = (self.ACCEL_X_MIN + self.ACCEL_X_MAX) / 2.0
                self.prefs.setSensor("ACCEL_X_OFFSET", offset)
                self.ACCEL_X_OFFSET = offset
                scale = self.GRAVITY / (self.ACCEL_X_MAX - self.ACCEL_X_OFFSET)
                self.prefs.setSensor("ACCEL_X_SCALE", scale)
                self.ACCEL_X_SCALE = scale

            self.kalmanAccelX = []

        avg = 0
        minY = 0
        maxY = 0
        offset = 0
        scale = 0
        if len(self.kalmanAccelY) is 200:
            for i in self.kalmanAccelY:
                avg += i
            avg = avg / len(self.kalmanAccelY)
            if avg > 240.0:
                maxY = avg
            if maxY > self.ACCEL_Y_MAX:
                self.prefs.setSensor("ACCEL_Y_MAX", maxY)
                self.ACCEL_Y_MAX = maxY
                print((self.ACCEL_Y_MAX))
                offset = (self.ACCEL_Y_MIN + self.ACCEL_Y_MAX) / 2.0
                self.prefs.setSensor("ACCEL_Y_OFFSET", offset)
                self.ACCEL_Y_OFFSET = offset
                scale = self.GRAVITY / (self.ACCEL_Y_MAX - self.ACCEL_Y_OFFSET)
                self.prefs.setSensor("ACCEL_Y_SCALE", scale)
                self.ACCEL_Y_SCALE = scale
            if avg < -240.0:
                minY = avg
            if minY < self.ACCEL_Y_MIN:
                self.prefs.setSensor("ACCEL_Y_MIN", minY)
                self.ACCEL_Y_MIN = minY
                print((self.ACCEL_Y_MIN))
                offset = (self.ACCEL_Y_MIN + self.ACCEL_Y_MAX) / 2.0
                self.prefs.setSensor("ACCEL_Y_OFFSET", offset)
                self.ACCEL_Y_OFFSET = offset
                scale = self.GRAVITY / (self.ACCEL_Y_MAX - self.ACCEL_Y_OFFSET)
                self.prefs.setSensor("ACCEL_Y_SCALE", scale)
                self.ACCEL_Y_SCALE = scale

            self.kalmanAccelY = []

        avg = 0
        minZ = 0
        maxZ = 0
        offset = 0
        scale = 0
        if len(self.kalmanAccelZ) is 200:
            for i in self.kalmanAccelZ:
                avg += i
            avg = avg / len(self.kalmanAccelZ)
            if avg > 240.0:
                maxZ = avg
            if maxZ > self.ACCEL_Z_MAX:
                self.prefs.setSensor("ACCEL_Z_MAX", maxZ)
                self.ACCEL_Z_MAX = maxZ
                print((self.ACCEL_Z_MAX))
                offset = (self.ACCEL_Z_MIN + self.ACCEL_Z_MAX) / 2.0
                self.prefs.setSensor("ACCEL_Z_OFFSET", offset)
                self.ACCEL_Z_OFFSET = offset
                scale = self.GRAVITY / (self.ACCEL_Z_MAX - self.ACCEL_Z_OFFSET)
                self.prefs.setSensor("ACCEL_Z_SCALE", scale)
                self.ACCEL_Z_SCALE = scale
            if avg < -240.0:
                minZ = avg
            if minZ < self.ACCEL_Z_MIN:
                self.prefs.setSensor("ACCEL_Z_MIN", minZ)
                self.ACCEL_Z_MIN = minZ
                print((self.ACCEL_Z_MIN))
                offset = (self.ACCEL_Z_MIN + self.ACCEL_Z_MAX) / 2.0
                self.prefs.setSensor("ACCEL_Z_OFFSET", offset)
                self.ACCEL_Z_OFFSET = offset
                scale = self.GRAVITY / (self.ACCEL_Z_MAX - self.ACCEL_Z_OFFSET)
                self.prefs.setSensor("ACCEL_Z_SCALE", scale)
                self.ACCEL_Z_SCALE = scale

            self.kalmanAccelZ = []

        """if len(self.measuredAccelX) is 200:
            pylab.plot(range(200), self.measuredAccelX, 'b', range(200), self.kalmanAccelX, 'g', range(200), self.measuredAccelY, 'r', range(200), self.kalmanAccelY, 'g', range(200), self.measuredAccelZ, 'y', range(200), self.kalmanAccelZ, 'g')
            pylab.xlabel('Time')
            pylab.ylabel('Value')
            pylab.title('Accelerometer with Kalman Filter')
            pylab.legend(('measured X', 'kalman X', 'measured Y', 'kalman Y', 'measured Z', 'kalman Z'))
            pylab.show()"""

        # Compensate magnetometer error
        #magnetom_tmp = []
        #for i in range(3):
            #magnetom_tmp.append(self.magnetom[i] - self.magn_ellipsoid_center[i])
        #self.magnetom = matrixVectorMultiply(self.magn_ellipsoid_transform, magnetom_tmp)

        temp = []
        self.measuredMagnetomX.append(self.magnetom[0])
        magnetomX = self.kfilterMagnetomX.GetCurrentState()[0, 0]
        #self.kalmanMagnetomX.append(magnetomX)
        self.kfilterMagnetomX.Step(numpy.matrix([0]), numpy.matrix([self.magnetom[0]]))
        temp.append((magnetomX - self.MAGN_X_OFFSET) * self.MAGN_X_SCALE)

        self.measuredMagnetomY.append(self.magnetom[1])
        magnetomY = self.kfilterMagnetomY.GetCurrentState()[0, 0]
        #self.kalmanMagnetomY.append(magnetomY)
        self.kfilterMagnetomY.Step(numpy.matrix([0]), numpy.matrix([self.magnetom[1]]))
        temp.append((magnetomY - self.MAGN_Y_OFFSET) * self.MAGN_Y_SCALE)

        self.measuredMagnetomZ.append(self.magnetom[2])
        magnetomZ = self.kfilterMagnetomZ.GetCurrentState()[0, 0]
        #self.kalmanMagnetomZ.append(magnetomZ)
        self.kfilterMagnetomZ.Step(numpy.matrix([0]), numpy.matrix([self.magnetom[2]]))
        temp.append((magnetomZ - self.MAGN_Z_OFFSET) * self.MAGN_Z_SCALE)

        self.magnetom = temp

        if len(self.measuredMagnetomX) == 1000 and len(self.measuredMagnetomY) == 1000 and len(self.measuredMagnetomZ) == 1000 and not self.magnetomCal:
            self.magnetomCal = True
            X = []
            Y = []
            Z = []
            for i in range(500):
                self.measuredMagnetomX.pop(i)
                self.measuredMagnetom.pop(i)
                self.measuredMagnetom.pop(i)

            for i in range(len(self.measuredMagnetomX)):
                X.append(self.measuredMagnetomX[i])
                Y.append(self.measuredMagnetomY[i])
                Z.append(self.measuredMagnetomZ[i])

            D = []
            for i in range(len(X)):
                temp = []
                temp.append(X[i] * X[i])
                temp.append(Y[i] * Y[i])
                temp.append(Z[i] * Z[i])
                temp.append(2 * X[i] * Y[i])
                temp.append(2 * X[i] * Z[i])
                temp.append(2 * Y[i] * Z[i])
                temp.append(2 * X[i])
                temp.append(2 * Y[i])
                temp.append(2 * Z[i])
                D.append(temp)

            D = numpy.matrix(D)
            X = numpy.matrix(X)
            Y = numpy.matrix(Y)
            Z = numpy.matrix(Z)

            temp1 = numpy.dot(D.transpose(), D)
            size = X.shape[1]
            temp2 = numpy.dot(D.transpose(), numpy.ones(size))
            v = numpy.dot(temp2, linalg.inv(temp1))
            v = numpy.array(v)[0]
            temp = []
            for i in v:
                temp.append(i)
            v = temp

            A = [[v[0], v[3], v[4], v[6]],
                 [v[3], v[1], v[5], v[7]],
                 [v[4], v[5], v[2], v[8]],
                 [v[6], v[7], v[8], -1.0]]

            A = numpy.matrix(A)

            temp1 = numpy.matrix([[v[0], v[3], v[4]], [v[3], v[1], v[5]], [v[4], v[5], v[2]]])
            temp2 = numpy.matrix([v[6], v[7], v[8]])

            center = numpy.dot(temp2, linalg.inv(temp1))
            center = numpy.dot(-1, center)

            center = numpy.array(center)[0]
            print((center))
            temp = []
            for i in center:
                temp.append(i)

            T = [[1, 0, 0, 0],
                 [0, 1, 0, 0],
                 [0, 0, 1, 0],
                 [temp[0], temp[1], temp[2], 1]]

            T = numpy.matrix(T)

            R = numpy.dot(T, A)
            R = numpy.dot(R, T.transpose())

            R1 = numpy.array(R)[0]
            R2 = numpy.array(R)[1]
            R3 = numpy.array(R)[2]
            R4 = numpy.array(R)[3]

            temp1 = []
            for i in R1:
                temp1.append(i)
            for i in R2:
                temp1.append(i)
            for i in R3:
                temp1.append(i)
            for i in R4:
                temp1.append(i)

            R = []
            temp2 = []
            for i in range(len(temp1)):
                temp2.append(temp1[i])
                if i == 3 or i == 7 or i == 11 or i == 15:
                    R.append(temp2)
                    temp2 = []

            negR = []
            extR = []
            count2 = 0
            for i in R:
                temp1 = []
                temp2 = []
                count1 = 0
                for j in i:
                    if count1 < 3:
                        temp1.append(-1 * j)
                        temp2.append(j)
                        count1 = count1 + 1
                if count2 < 3:
                    negR.append(temp1)
                    extR.append(temp1)
                    count2 = count2 + 1

            negR = numpy.matrix(negR)
            extR = numpy.matrix(extR)
            evd = numpy.dot(negR, linalg.inv(extR))
            evd = linalg.eig(evd)

            #Array of Real Values from Eigen Problem
            evecs = []
            evd1 = evd[1]
            temp1 = []
            temp2 = []
            temp3 = []

            for i in evd1:
                temp = numpy.array(i)[0]
                temp1.append(temp)
                temp = numpy.array(i)[1]
                temp2.append(temp)
                temp = numpy.array(i)[2]
                temp3.append(temp)

            evecs1 = []
            evecs2 = []
            evecs3 = []
            for i in range(len(temp1)):
                evecs1.append(temp1[i])
                evecs2.append(temp2[i])
                evecs3.append(temp3[i])

            evecs.append(evecs1)
            evecs.append(evecs2)
            evecs.append(evecs3)

            print((evecs))

        """if len(self.measuredMagnetomX) is 200:
            pylab.plot(range(200), self.measuredMagnetomX, 'b', range(200), self.kalmanMagnetomX, 'g', range(200), self.measuredMagnetomY, 'r', range(200), self.kalmanMagnetomY, 'g', range(200), self.measuredMagnetomZ, 'y', range(200), self.kalmanMagnetomZ, 'g')
            pylab.xlabel('Time')
            pylab.ylabel('Value')
            pylab.title('Magnetometer with Kalman Filter')
            pylab.legend(('measured X', 'kalman X', 'measured Y', 'kalman Y', 'measured Z', 'kalman Z'))
            pylab.show()"""

        # Compensate gyroscope error
        temp = []
        if self.gyro[0] == 1.0 or self.gyro[0] < -10000:
            self.gyro.pop(0)
            try:
                self.gyro.insert(0, self.kalmanGyroX[-1])
            except:
                pass
        self.measuredGyroX.append(self.gyro[0])
        gyroX = self.kfilterGyroX.GetCurrentState()[0, 0]
        self.kalmanGyroX.append(gyroX)
        self.kfilterGyroX.Step(numpy.matrix([0]), numpy.matrix([self.gyro[0]]))
        temp.append(gyroX - self.GYRO_AVERAGE_OFFSET_X)

        if self.gyro[1] == 1.0 or self.gyro[1] < -1000:
            self.gyro.pop(1)
            try:
                self.gyro.insert(1, self.kalmanGyroY[-1])
            except:
                pass
        self.measuredGyroY.append(self.gyro[1])
        gyroY = self.kfilterGyroY.GetCurrentState()[0, 0]
        self.kalmanGyroY.append(gyroY)
        self.kfilterGyroY.Step(numpy.matrix([0]), numpy.matrix([self.gyro[1]]))
        temp.append(gyroY - self.GYRO_AVERAGE_OFFSET_Y)

        if self.gyro[2] == 1.0 or self.gyro[2] < -10000:
            self.gyro.pop(2)
            try:
                self.gyro.insert(2, self.kalmanGyroZ[-1])
            except:
                pass
        self.measuredGyroZ.append(self.gyro[2])
        gyroZ = self.kfilterGyroZ.GetCurrentState()[0, 0]
        self.kalmanGyroZ.append(gyroZ)
        self.kfilterGyroZ.Step(numpy.matrix([0]), numpy.matrix([self.gyro[2]]))
        temp.append(gyroZ - self.GYRO_AVERAGE_OFFSET_Z)

        self.gyro = temp

        if len(self.kalmanGyroX) is 201:
            self.kalmanGyroX.pop(0)
            self.measuredGyroX.pop(0)

        if len(self.kalmanGyroY) is 201:
            self.kalmanGyroY.pop(0)
            self.measuredGyroY.pop(0)

        if len(self.kalmanGyroZ) is 201:
            self.kalmanGyroZ.pop(0)
            self.measuredGyroZ.pop(0)

        """if len(self.measuredGyroX) is 200:
            pylab.plot(range(200), self.measuredGyroX, 'b', range(200), self.kalmanGyroX, 'g', range(200), self.measuredGyroY, 'r', range(200), self.kalmanGyroY, 'g', range(200), self.measuredGyroZ, 'y', range(200), self.kalmanGyroZ, 'g')
            pylab.xlabel('Time')
            pylab.ylabel('Value')
            pylab.title('Gyro with Kalman Filter')
            pylab.legend(('measured X', "", 'measured Y', "", 'measured Z', 'kalman '))
            pylab.show()"""

    def getVelocity(self):
        temp = []
        for i in self.accel:
            temp.append(i * self.G_Dt)
        for i in range(len(self.velocity)):
            tempA = self.velocity[i] + temp[i]
            self.velocity.pop(i)
            self.velocity.insert(i, tempA)
        #print((self.velocity))
        #print((self.accel))

    def matrixUpdate(self):
        self.Gyro_Vector = []
        self.Gyro_Vector.append(self.gyroScaledRAD(self.gyro[0]))  # gyro x roll
        self.Gyro_Vector.append(self.gyroScaledRAD(self.gyro[1]))  # gyro y pitch
        self.Gyro_Vector.append(self.gyroScaledRAD(self.gyro[2]))  # gyro z yaw

        self.Accel_Vector = []
        self.Accel_Vector.append(self.accel[0])
        self.Accel_Vector.append(self.accel[1])
        self.Accel_Vector.append(self.accel[2])

        self.Omega = self.vectorAdd(self.Gyro_Vector, self.Omega_P)  # adding proportional term
        self.Omega_Vector = self.vectorAdd(self.Omega, self.Omega_I)  # adding Integrator term

        temp1 = []
        temp2 = []
        temp3 = []
        temp1.append(0)
        temp1.append(-self.G_Dt * self.Omega_Vector[2])  # -z
        temp1.append(self.G_Dt * self.Omega_Vector[1])  # y

        temp2.append(self.G_Dt * self.Omega_Vector[2])  # z
        temp2.append(0)
        temp2.append(-self.G_Dt * self.Omega_Vector[0])  # -x

        temp3.append(-self.G_Dt * self.Omega_Vector[1])  # -y
        temp3.append(self.G_Dt * self.Omega_Vector[0])  # x
        temp3.append(0)

        self.Update_Matrix = []
        self.Update_Matrix.append(temp1)
        self.Update_Matrix.append(temp2)
        self.Update_Matrix.append(temp3)

        self.Temporary_Matrix = self.matrixMultiply(self.DCM_Matrix, self.Update_Matrix)  # A * B = C

        tempDCM = []
        for x in range(3):  # Matrix Addition (update)
            temp = []
            for y in range(3):
                temp.append(self.DCM_Matrix[x][y] + self.Temporary_Matrix[x][y])
            tempDCM.append(temp)
        self.DCM_Matrix = tempDCM

    def normalize(self):
        # print((self.DCM_Matrix))
        temporary = []
        error = -self.vectorDotProduct(self.DCM_Matrix[0], self.DCM_Matrix[1]) * 0.5  # eq.19

        temporary.append(self.vectorScale(self.DCM_Matrix[1], error))  # eq.19
        temporary.append(self.vectorScale(self.DCM_Matrix[0], error))  # eq.19

        x = self.vectorAdd(temporary[0], self.DCM_Matrix[0])
        temporary.pop(0)
        temporary.insert(0, x)  # eq.19
        x = self.vectorAdd(temporary[1], self.DCM_Matrix[1])
        temporary.pop(1)
        temporary.insert(1, x)  # eq.19

        temporary.append(self.vectorCrossProduct(temporary[0], temporary[1]))  # c = a x b  eq.20

        # print((temporary))

        renorm = 0.5 * (3 - (self.vectorDotProduct(temporary[0], temporary[0])))  # eq.21
        tempDCM = []
        tempDCM.append(self.vectorScale(temporary[0], renorm))

        renorm = 0.5 * (3 - (self.vectorDotProduct(temporary[1], temporary[1])))  # eq.21
        tempDCM.append(self.vectorScale(temporary[1], renorm))

        renorm = 0.5 * (3 - (self.vectorDotProduct(temporary[2], temporary[2])))  # eq.21
        tempDCM.append(self.vectorScale(temporary[2], renorm))

    def driftCorrection(self):
        #Compensation the Roll, Pitch and Yaw drift.
        Scaled_Omega_P = []
        Scaled_Omega_I = []

        #*****Roll and Pitch***************

        # Calculate the magnitude of the accelerometer vector
        Accel_magnitude = math.sqrt(self.Accel_Vector[0] * self.Accel_Vector[0] + self.Accel_Vector[1] * self.Accel_Vector[1] + self.Accel_Vector[2] * self.Accel_Vector[2])
        Accel_magnitude = Accel_magnitude / self.GRAVITY  # Scale to gravity.

        # Dynamic weighting of accelerometer info (reliability filter)
        # Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
        Accel_weight = sorted([1 - 2 * math.fabs(1 - Accel_magnitude), 0, 1])[1]
        self.errorRollPitch = self.vectorCrossProduct(self.Accel_Vector, self.DCM_Matrix[2])  # adjust the ground of reference
        self.Omega_P = self.vectorScale(self.errorRollPitch, (self.Kp_ROLLPITCH * Accel_weight))

        Scaled_Omega_I = self.vectorScale(self.errorRollPitch, self.Ki_ROLLPITCH * Accel_weight)
        self.Omega_I = self.vectorAdd(self.Omega_I, Scaled_Omega_I)

        #*****YAW***************
        # We make the gyro YAW drift correction based on compass magnetic headin
        mag_heading_x = math.cos(self.MAG_Heading)
        mag_heading_y = math.sin(self.MAG_Heading)
        errorCourse = (self.DCM_Matrix[0][0] * mag_heading_y) - (self.DCM_Matrix[1][0] * mag_heading_x)  # Calculating YAW error
        self.errorYaw = self.vectorScale(self.DCM_Matrix[2], errorCourse)  # Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

        Scaled_Omega_P = self.vectorScale(self.errorYaw, self.Kp_YAW)  # .01proportional of YAW.
        self.Omega_P = self.vectorAdd(self.Omega_P, Scaled_Omega_P)  # Adding  Proportional.

        Scaled_Omega_I = self.vectorScale(self.errorYaw, self.Ki_YAW)  # .00001Integrator
        self.Omega_I = self.vectorAdd(self.Omega_I, Scaled_Omega_I)  # adding integrator to the Omega_I

    def eulerAngles(self):
        if self.DCM_Matrix[2][0] > -1 and self.DCM_Matrix[2][0] < 1:
            # print((self.DCM_Matrix[2][0]))
            self.pitch = -math.asin(self.DCM_Matrix[2][0])
            self.roll = -math.atan2(self.DCM_Matrix[2][1], self.DCM_Matrix[2][2])
            self.yaw = math.atan2(self.DCM_Matrix[1][0], self.DCM_Matrix[0][0])
            self.roll = -1 * self.roll

        else:
            self.resetSensorFusion()

    def compassHeading(self):
        cos_roll = math.cos(self.roll)
        sin_roll = math.sin(self.roll)
        cos_pitch = math.cos(self.pitch)
        sin_pitch = math.sin(self.pitch)

        # Tilt compensated magnetic field X
        mag_x = self.magnetom[0] * cos_pitch + self.magnetom[1] * sin_roll * sin_pitch + self.magnetom[2] * cos_roll * sin_pitch

        # Tilt compensated magnetic field Y
        mag_y = self.magnetom[1] * cos_roll - self.magnetom[2] * sin_roll

        # Magnetic Heading
        self.MAG_Heading = math.atan2(-mag_y, mag_x)
        if self.MAG_Heading < 0:
            self.MAG_Heading = math.pi + self.MAG_Heading
        else:
            self.MAG_Heading = -1 * (math.pi - self.MAG_Heading)

    def getYPR(self):
        return self.yaw, self.pitch, self.roll

    def toRAD(self, x):
        rad = x * (math.pi / 180)
        return rad

    def toDEG(self, x):
        deg = x * (180 / math.pi)
        return deg

    def gyroScaledRAD(self, x):
        scaledRAD = x * self.toRAD(self.GYRO_GAIN)  # Calculate the scaled gyro readings in radians per second
        return scaledRAD

    def setScaleAndOffsetValues(self):
        # Sensor calibration scale and offset values
        self.ACCEL_X_OFFSET = (self.ACCEL_X_MIN + self.ACCEL_X_MAX) / 2.0
        self.ACCEL_Y_OFFSET = (self.ACCEL_Y_MIN + self.ACCEL_Y_MAX) / 2.0
        self.ACCEL_Z_OFFSET = (self.ACCEL_Z_MIN + self.ACCEL_Z_MAX) / 2.0
        self.ACCEL_X_SCALE = self.GRAVITY / (self.ACCEL_X_MAX - self.ACCEL_X_OFFSET)
        self.ACCEL_Y_SCALE = self.GRAVITY / (self.ACCEL_Y_MAX - self.ACCEL_Y_OFFSET)
        self.ACCEL_Z_SCALE = self.GRAVITY / (self.ACCEL_Z_MAX - self.ACCEL_Z_OFFSET)

        self.MAGN_X_OFFSET = (self.MAGN_X_MIN + self.MAGN_X_MAX) / 2.0
        self.MAGN_Y_OFFSET = (self.MAGN_Y_MIN + self.MAGN_Y_MAX) / 2.0
        self.MAGN_Z_OFFSET = (self.MAGN_Z_MIN + self.MAGN_Z_MAX) / 2.0
        self.MAGN_X_SCALE = 100.0 / (self.MAGN_X_MAX - self.MAGN_X_OFFSET)
        self.MAGN_Y_SCALE = 100.0 / (self.MAGN_Y_MAX - self.MAGN_Y_OFFSET)
        self.MAGN_Z_SCALE = 100.0 / (self.MAGN_Z_MAX - self.MAGN_Z_OFFSET)

    # Computes the dot product of two vectors
    def vectorDotProduct(self, vector1, vector2):
        result = 0

        for i in range(3):
            result += vector1[i] * vector2[i]

        return result

    # Computes the cross product of two vectors
    # out has to different from vector1 and vector2 (no in-place)!
    def vectorCrossProduct(self, vector1, vector2):
        temp = []
        temp.append((vector1[1] * vector2[2]) - (vector1[2] * vector2[1]))  # (A * B)x
        temp.append((vector1[2] * vector2[0]) - (vector1[0] * vector2[2]))  # (A * B)y
        temp.append((vector1[0] * vector2[1]) - (vector1[1] * vector2[0]))  # (A * B)z
        return temp

    # Multiply the vector by a scalar
    def vectorScale(self, vector, scale):
        temp = []
        for i in range(3):
            temp.append(vector[i] * scale)
        return temp

    # Adds two vectors
    def vectorAdd(self, vector1, vector2):
        temp = []
        for i in range(3):
            temp.append(vector1[i] + vector2[i])
        return temp

    # Multiply two 3x3 matrices: out = a * b
    # out has to different from a and b (no in-place)!
    def matrixMultiply(self, a, b):
        temp = []
        for x in range(3):  # rows
            tempSub = []
            for y in range(3):  # columns
                tempSub.append(a[x][0] * b[0][y] + a[x][1] * b[1][y] + a[x][2] * b[2][y])
            temp.append(tempSub)
        return temp

    # Multiply 3x3 matrix with vector: out = a * b
    # out has to different from b (no in-place)!
    def matrixVectorMultiply(self, a, b):
        temp = []
        for x in range(3):
            temp.append(a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2])
        return temp

    # Init rotation matrix using euler angles
    def initRotationMatrix(self, yaw, pitch, roll):
        temp = []
        c1 = math.cos(roll)
        s1 = math.sin(roll)
        c2 = math.cos(pitch)
        s2 = math.sin(pitch)
        c3 = math.cos(yaw)
        s3 = math.sin(yaw)

        # Euler angles, right-handed, intrinsic, XYZ convention
        # (which means: rotate around body axes Z, Y', X'')
        m0 = []
        calc1 = c2 * c3
        calc2 = c3 * s1 * s2 - c1 * s3
        calc3 = s1 * s3 + c1 * c3 * s2
        m0.append(calc1)
        m0.append(calc2)
        m0.append(calc3)

        m1 = []
        calc1 = c2 * s3
        calc2 = c1 * c3 + s1 * s2 * s3
        calc3 = c1 * s2 * s3 - c3 * s1
        m1.append(calc1)
        m1.append(calc2)
        m1.append(calc3)

        m2 = []
        calc1 = -s2
        calc2 = c2 * s1
        calc3 = c1 * c2
        m2.append(calc1)
        m2.append(calc2)
        m2.append(calc3)

        temp.append(m0)
        temp.append(m1)
        temp.append(m2)

        return temp
