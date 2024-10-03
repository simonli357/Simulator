#!/usr/bin/env python3

# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

import serial
from filehandler      import FileHandler
from messageconverter import MessageConverter
import json

import rospy
import argparse

from std_msgs.msg      import String

class serialNODE():
    def __init__(self, ns = "automobile", rate = 19200):
        """It forwards the control messages received from socket to the serial handling node. 
        """
        devFile = '/dev/ttyACM0'
        logFile = 'historyFile.txt'
        
        # comm init       
        self.serialCom = serial.Serial(devFile,rate,timeout=0.1)
        self.serialCom.flushInput()
        self.serialCom.flushOutput()

        # log file init
        self.historyFile = FileHandler(logFile)
        
        # message converted init
        self.messageConverter = MessageConverter()
        
        self.buff=""
        self.isResponse=False
        self.__subscribers={}
        
        rospy.init_node('serialNODE', anonymous=False)
        self.rate = rospy.Rate(25)
        
        self.command_subscriber = rospy.Subscriber('/' + ns + '/command', String, self._write, queue_size=1)
            
     # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads
        """
        rospy.loginfo("starting serialNODE")
        self._read()    
        
    # ===================================== READ ==========================================
    def _read(self):
        """ It's represent the reading activity on the the serial.
        """
        while not rospy.is_shutdown():
            # read_chr=self.serialCom.read()
            # print(read_chr)
            # try:
            #     read_chr=(read_chr.decode("ascii"))
            #     if read_chr=='@':
            #         self.isResponse=True
            #         if len(self.buff)!=0:
            #             self.__checkSubscriber(self.buff)
            #         self.buff=""
            #     elif read_chr=='\r':   
            #         self.isResponse=False
            #         if len(self.buff)!=0:
            #             self.__checkSubscriber(self.buff)
            #         self.buff=""
            #     if self.isResponse:
            #         self.buff+=read_chr
            #     self.historyFile.write(read_chr)
                 
            # # except UnicodeDecodeError:
            # #     pass
            # except:
            #     pass
            rospy.spin()
    
    def __checkSubscriber(self,f_response):
        """ Checking the list of the waiting object to redirectionate the message to them. 
        """
        l_key=f_response[1:5]
        if l_key in self.__subscribers:
            subscribers = self.__subscribers[l_key]
            for sub in subscribers:
                sub.publish(f_response)
        
    # ===================================== WRITE ==========================================
    def _write(self, msg):
        """ Represents the writing activity on the the serial.
        """
        command = json.loads(msg.data)
        # print("hh", type(command), type(msg), type(msg.data))
        # print(msg)
        # print(command)
        # Unpacking the dictionary into action and values
        command_msg = self.messageConverter.get_command(**command)
        # command_msg = '#7:1;;'
        # print(command_msg)
        self.serialCom.write(command_msg.encode('ascii'))
        # self.historyFile.write(command_msg)
        # self.rate.sleep()
            
if __name__ == "__main__":
    # add an argument for namespace name, default is automobile
    parser = argparse.ArgumentParser()
    parser.add_argument("--ns", type=str, default="automobile", help="namespace name")
    parser.add_argument("--baudrate", type=str, default=19200, help="serial baudrate")
    args = parser.parse_args(rospy.myargv()[1:])
    try:
        serNod = serialNODE(args.ns, int(args.baudrate))
        serNod.run()
    except rospy.ROSInterruptException:
        pass