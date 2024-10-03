#!/usr/bin/env python3

import sys
sys.path.append('.')
import os.path
import time
import math
import serial

import rospy

# from utils.msg import encoder
from std_msgs.msg import Float32, Header

class encoderNODE():
    def __init__(self):
        rospy.init_node('encoderNODE', anonymous=False)

        self.ser = serial.Serial(
            # port='/dev/serial_device',
            port='/dev/ttyACM0',
            baudrate = 19200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

        # encoder publisher object
        self.rate = rospy.Rate(30)
        # self.encoder_publisher = rospy.Publisher("/automobile/encoder", encoder, queue_size=3)

    #================================ RUN ========================================
    def run(self):
        rospy.loginfo("starting encoderNODE")
        self._getting()

    #================================ GETTING ========================================
    def _getting(self):
        while not rospy.is_shutdown():
            # encoderdata = encoder()
            try:
                speed = self.ser.readline() #make sure this is a float32
                s = speed.decode("utf-8")
                print(s)
                colon_index = s.find(':')
                subs = s[colon_index+1:s.find(';', colon_index)]
                f = float(subs)
                # print(f)w
                # encoderdata.speed  = f
                # header = Header()
                # header.stamp = rospy.Time.now()
                # header.frame_id = "bno055"
                # encoderdata.header = header
            except:
                pass
            # self.encoder_publisher.publish(encoderdata)
            
            # time.sleep(0.1)
        
if __name__ == "__main__":
    encoderNod = encoderNODE()
    encoderNod.run()