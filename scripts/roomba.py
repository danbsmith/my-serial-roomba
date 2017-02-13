#!/usr/bin/env python

import serial
import rospy
import time
from roomba_serial.msg import *
from roomba_serial.srv import *

currmode = 0

class SerialRoomba:
    createtime = time.time()
    serialport = serial.Serial()
    def __init__(self, port):
        global currmode
        rospy.loginfo("Getting serial port")
        self.serialport = serial.Serial(port, baudrate = 115200, timeout = 10)
        self.wakeSCI()
        rospy.loginfo("Initialized serial interface with Roomba")

    def sendcommand(self, data):
        global currmode
        rospy.loginfo("Sending command %u, current mode is %d", data[0], currmode)
        if((time.time() - self.createtime) > 300.0):
            self.wakeSCI()
            self.createtime = time.time()
        self.serialport.write(data)
        return

    def getreply(self, size):
        reply = bytearray(self.serialport.read(size))
        return reply

    def setbaud(self, rate):
        self.serialport.setBaudRate(rate)
        return

    def wakeSCI(self): 
        global currmode
        oldmode = currmode
        rospy.loginfo("Beginning SCI Wake.")
        self.serialport.setRTS(0)
        time.sleep(0.5)
        self.serialport.setRTS(1)
        time.sleep(0.5)
        self.serialport.write(bytearray([chr(128)]))
        time.sleep(0.2)
        if oldmode > 1:
            self.serialport.write(bytearray([chr(129 + oldmode)]))
        currmode = oldmode
        rospy.loginfo("Woke SCI, set currmode to %d", oldmode)

controller = SerialRoomba("/dev/ttyUSB0")

def ModeCallBack(data):
    modecode = data.modecode
    global currmode
    if modecode == 0:
        cmd = bytearray([chr(133)])
        currmode = 0
    elif modecode == 2:
        if currmode == 1:
            cmd = bytearray([chr(131)])
            currmode = 2
        elif currmode == 3:
            cmd = bytearray([chr(131)])
            currmode = 2
        elif currmode == 2:
            cmd = bytearray([chr(131)])
            return
        elif currmode == 0:
            rospy.loginfo("Can't enter safe mode without starting SCI!")
            return
    elif modecode == 1:
        cmd = bytearray([chr(128)])
        currmode = 1
    elif modecode == 3:
        cmd = bytearray([chr(132)])
    else:
        rospy.loginfo("Bad mode code passed to serial controller")
        return
    global controller
    controller.sendcommand(cmd)
    return

def BaudCallBack(data):
    cmd = bytearray([chr(129), chr(data.baudcode)])
    global controller
    controller.sendcommand(cmd)
    if data.baudcode == 0:
        controller.setbaud(300)
    elif data.baudcode == 1:
        controller.setbaud(600)
    elif data.baudcode == 2:
        controller.setbaud(1200)
    elif data.baudcode == 3:
        controller.setbaud(2400)
    elif data.baudcode == 4:
        controller.setbaud(4800)
    elif data.baudcode == 5:
        controller.setbaud(9600)
    elif data.baudcode == 6:
        controller.setbaud(14400)
    elif data.baudcode == 7:
        controller.setbaud(19200)
    elif data.baudcode == 8:
        controller.setbaud(28800)
    elif data.baudcode == 9:
        controller.setbaud(38400)
    elif data.baudcode == 10:
        controller.setbaud(57600)
    elif data.baudcode == 11:
        controller.setbaud(115200)
    else:
        rospy.loginfo("Could not set baud rate; invalid code")
        return
    time.sleep(0.1)
    return

def DriveRoombaCallBack(data):
    radius = twoscomplement(data.radius)
    velocity = twoscomplement(data.velocity)
    cmd = bytearray([chr(137), chr(velocity[0]), chr(velocity[1]), chr(radius[0]), chr(radius[1])])
    global controller
    controller.sendcommand(cmd)
    return

def SendButtonCallBack(data):
    global controller
    controller.sendcommand(bytearray([chr(data.buttoncode + 133)]))
    return

def twoscomplement(num):
    return (((num & 0xFF00)>>8), (num & 0xFF))

def revtwoscomplement(num):
    if num[0] >= 128:
        return (((num[0] << 8) | num[1]) - 65536)
    else:
        return((num[0] << 8) | num[1])

def handle_sensor_request(data):
    if data.request == 0:
        incoming = 26
    elif data.request == 1:
        incoming = 10
    elif data.request == 2:
        incoming = 6
    elif data.request == 3:
        incoming = 10
    else:
        rospy.loginfo("Received invalid sensor subset code.")
    global controller
    controller.sendcommand(bytearray([142, data.request]))
    sensors = controller.getreply(incoming)
    resp = SensorsResponse()
    if data.request == 0:
        if sensors[0] & 16 == 16:
            resp.caster_down = True
        else:
            resp.caster_down = False
        if sensors[0] & 8 == 8:
            resp.left_wheel_down = True
        else:
            resp.left_wheel_down = False
        if sensors[0] & 4 == 4:
            resp.right_wheel_down = True
        else:
            resp.right_wheel_down = False
        if sensors[0] & 2 == 2:
            resp.lbump = True
        else:
            resp.lbump = False
        if sensors[0] & 1 == 1:
            resp.rbump = True
        else:
            resp.rbump = False
        if sensors[1] == 1:
            resp.wall = True
        else:
            resp.wall = False
        if sensors[2] == 1:
            resp.lcliff = True
        else:
            resp.lcliff = False
        if sensors[3] == 1:
            resp.lfcliff = True
        else:
            resp.lfcliff = False
        if sensors[4] == 1:
            resp.rcliff = True
        else:
            resp.rcliff = False
        if sensors[5] == 1:
            resp.rfcliff = True
        else:
            resp.rfcliff = False
        if sensors[6] == 1:
            resp.vwall = True
        else:
            resp.vwall = False
        if sensors[7] & 16 == 16:
            resp.lmotoroverload = True
        else:
            resp.lmotoroverload = False
        if sensors[7]& 8 == 8:
            resp.rmotoroverload = True
        else:
            resp.rmotoroverload = False
        if sensors[7] & 4 == 4:
            resp.brushmotoroverload = True
        else:
            resp.brushmotoroverload = False
        if sensors[7] & 2 == 2:
            resp.vacmotoroverload = True
        else:
            resp.vacmotoroverload = False
        if sensors[7] & 1 == 1:
            resp.sidemotoroverload = True
        else:
            resp.sidemotoroverload = False
        resp.ldirt = sensors[8]
        resp.rdirt = sensors[9]
        resp.remote = sensors[10]
        if sensors[11] & 8 == 8:
            resp.powerbutton = True
        else:
            resp.powerbutton = False
        if sensors[11] & 4 == 4:
            resp.spotbutton = True
        else:
            resp.spotbutton = False
        if sensors[11] & 2 == 2:
            resp.cleanbutton = True
        else:
            resp.cleanbutton = False
        if sensors[11] & 1 == 1:
            resp.maxbutton = True
        else:
            resp.maxbutton = False
        resp.distance = revtwoscomplement(bytearray([sensors[12], sensors[13]]))
        resp.angle = 360.0 * revtwoscomplement(bytearray([sensors[14], sensors[15]])) / (258 * 3.1415926)
        resp.chargestate = sensors[16]
        resp.voltage = (256 * sensors[17]) + sensors[18]
        resp.current = revtwoscomplement(bytearray([sensors[19], sensors[20]]))
        if sensors[21] >= 128:
            resp.temperature = sensors[21] - 128
        else:
            resp.temperature = sensors[21]
        resp.charge = (256 * sensors[22]) + sensors[23]
        resp.capacity = (256 * sensors[24]) + sensors[25]
    elif data.request == 1:
        if sensors[0] & 16 == 16:
            resp.caster_down = True
        else:
            resp.caster_down = False
        if sensors[0] & 8 == 8:
            resp.left_wheel_down = True
        else:
            resp.left_wheel_down = False
        if sensors[0] & 4 == 4:
            resp.right_wheel_down = True
        else:
            resp.right_wheel_down = False
        if sensors[0] & 2 == 2:
            resp.lbump = True
        else:
            resp.lbump = False
        if sensors[0] & 1 == 1:
            resp.rbump = True
        else:
            resp.rbump = False
        if sensors[1] == 1:
            resp.wall = True
        else:
            resp.wall = False
        if sensors[2] == 1:
            resp.lcliff = True
        else:
            resp.lcliff = False
        if sensors[3] == 1:
            resp.lfcliff = True
        else:
            resp.lfcliff = False
        if sensors[4] == 1:
            resp.rcliff = True
        else:
            resp.rcliff = False
        if sensors[5] == 1:
            resp.rfcliff = True
        else:
            resp.rfcliff = False
        if sensors[6] == 1:
            resp.vwall = True
        else:
            resp.vwall = False
        if sensors[7] & 16 == 16:
            resp.lmotoroverload = True
        else:
            resp.lmotoroverload = False
        if sensors[7]& 8 == 8:
            resp.rmotoroverload = True
        else:
            resp.rmotoroverload = False
        if sensors[7] & 4 == 4:
            resp.brushmotoroverload = True
        else:
            resp.brushmotoroverload = False
        if sensors[7] & 2 == 2:
            resp.vacmotoroverload = True
        else:
            resp.vacmotoroverload = False
        if sensors[7] & 1 == 1:
            resp.sidemotoroverload = True
        else:
            resp.sidemotoroverload = False
        resp.ldirt = sensors[8]
        resp.rdirt = sensors[9]
    elif data.request == 2:
        resp.remote = sensors[0]
        if sensors[1] & 8 == 8:
            resp.powerbutton = True
        else:
            resp.powerbutton = False
        if sensors[1] & 4 == 4:
            resp.spotbutton = True
        else:
            resp.spotbutton = False
        if sensors[1] & 2 == 2:
            resp.cleanbutton = True
        else:
            resp.cleanbutton = False
        if sensors[1] & 1 == 1:
            resp.maxbutton = True
        else:
            resp.maxbutton = False
        resp.distance = revtwoscomplement(bytearray([sensors[2], sensors[3]]))
        resp.angle = 360.0 * revtwoscomplement(bytearray([sensors[4], sensors[5]])) / (258 * 3.1415926)
    elif data.request == 3:
        resp.chargestate = sensors[0]
        resp.voltage = (256 * sensors[1]) + sensors[2]
        resp.current = revtwoscomplement(bytearray([sensors[3], sensors[4]]))
        if sensors[5] >= 128:
            resp.temperature = sensors[5] - 128
        else:
            resp.temperature = sensors[5]
        resp.charge = (256 * sensors[6]) + sensors[7]
        resp.capacity = (256 * sensors[8]) + sensors[9]
    return resp

def serial_controller():
    rospy.init_node("roomba_controller", anonymous=True)
    global currmode
    rospy.Subscriber("BUTTON_OUT", SendButton, SendButtonCallBack)
    rospy.Subscriber("DRIVE_CMDS", DriveRoomba, DriveRoombaCallBack)
    rospy.Subscriber("BAUD_CHANGES", SetBaud, BaudCallBack)
    rospy.Subscriber("MODE_CHANGES", SetMode, ModeCallBack)
    sensor_service = rospy.Service('GetSensors', Sensors, handle_sensor_request)
    rospy.spin()

if __name__ == '__main__':
    serial_controller()
