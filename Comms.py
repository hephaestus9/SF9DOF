# -*- coding: utf-8 -*-
import socket
import threading
import ast

try:
    from . import DCM
    from .config import preferences
except:
    import DCM
    from config import preferences


class Comms():

    def __init__(self):
        self.sensorData = None
        self.prefs = preferences.Prefs()
        self.UDP_IPSend = self.prefs.getRov("CONTROL_UDP_IPSend")
        self.UDP_IPReceive = self.prefs.getRov("CONTROL_UDP_IPReceive")
        self.UDP_PORT_SEND = int(self.prefs.getRov("CONTROL_UDP_PORT_SEND"))
        self.UDP_PORT_RECV = int(self.prefs.getRov("CONTROL_UDP_PORT_RECV"))
        self.dcm = None
        self.ypr = []

    def printInput(self, input):
        print((input))

    def sendSetup(self):
        self.sendSock = socket.socket(socket.AF_INET,  # Internet
                             socket.SOCK_DGRAM)  # UDP

    def sendUDP(self, data):
        self.sendSock.sendto(data, (self.UDP_IPSend, self.UDP_PORT_SEND))

    def receiverSetup(self):
        self.dcm = DCM.DCM()
        self.receiveSock = socket.socket(socket.AF_INET,  # Internet
                         socket.SOCK_DGRAM)  # UDP

        self.receiveSock.bind((self.UDP_IPReceive, self.UDP_PORT_RECV))

    def receive(self):
        def receiverListen(sock, telemetry):
            while True:
                data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
                telemetry(data)
                # print((data))

        receiver_thread = threading.Thread(target=receiverListen, args=(self.receiveSock, self.telemetry))
        receiver_thread.daemon = True
        receiver_thread.start()

    def getSensorData(self):
        return self.sensorData

    def getYPR(self):
        return self.ypr

    def telemetry(self, data):
        data = ast.literal_eval(data)
        for key, val in list(data.items()):
            if "sensor stick" in key:
                self.sensorData = data["sensor stick"]
                self.ypr = self.dcm.setSensorData(self.sensorData)
            if "depth sensor" in key:
                pass
            if "altimeter" in key:
                pass