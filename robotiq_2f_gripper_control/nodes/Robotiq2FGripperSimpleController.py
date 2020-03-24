#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
Command-line interface for sending simple commands to a ROS node controlling a 2F gripper.

This serves as an example for publishing messages on the 'Robotiq2FGripperRobotOutput' topic using the 'Robotiq2FGripper_robot_output' msg type for sending commands to a 2F gripper.
"""

import rclpy
from rclpy.node import Node
from robotiq_2f_gripper_control.msg import Robotiq2FGripperOutput  as outputMsg
from time import sleep


def genCommand(char, command):
    """Update the command according to the character entered by the user."""    
        
    if char == 'a':
        command = outputMsg()
        command.r_act = 1
        command.r_gto = 1
        command.r_sp  = 255
        command.r_fr  = 150

    if char == 'r':
        command = outputMsg()
        command.r_act = 0

    if char == 'c':
        command.r_pr = 255

    if char == 'o':
        command.r_pr = 0   

    #If the command entered is a int, assign this value to r_prA
    try: 
        if int(char) > 255:
            command.r_pr = 255
        elif int(char) < 0:
            command.r_pr = 0
        else:
            command.r_pr = int(char)
    except ValueError:
        pass                    
        
    if char == 'f':
        if (command.r_sp + 25) > 255:
            command.r_sp = 255
        else:
            command.r_sp += 25
            
    if char == 'l':
        if (command.r_sp - 25) < 0:
            command.r_sp = 0
        else:
            command.r_sp -= 25
            
    if char == 'i':
        if (command.r_fr + 25) > 255:
            command.r_fr = 255
        else:
            command.r_fr += 25
            
    if char == 'd':
        if (command.r_fr - 25) < 0:
            command.r_fr = 0
        else:
            command.r_fr -= 25

    return command
        

def askForCommand(command):
    """Ask the user for a command to send to the gripper."""    

    currentCommand  = 'Simple 2F Gripper Controller\n-----\nCurrent command:'
    currentCommand += '  r_act = '  + str(command.r_act)
    currentCommand += ', r_gto = '  + str(command.r_gto)
    currentCommand += ', r_atr = '  + str(command.r_atr)
    currentCommand += ', r_pr = '   + str(command.r_pr )
    currentCommand += ', r_sp = '   + str(command.r_sp )
    currentCommand += ', r_fr = '   + str(command.r_fr )

    print(currentCommand)

    strAskForCommand  = '-----\nAvailable commands\n\n'
    strAskForCommand += 'r: Reset\n'
    strAskForCommand += 'a: Activate\n'
    strAskForCommand += 'c: Close\n'
    strAskForCommand += 'o: Open\n'
    strAskForCommand += '(0-255): Go to that position\n'
    strAskForCommand += 'f: Faster\n'
    strAskForCommand += 'l: Slower\n'
    strAskForCommand += 'i: Increase force\n'
    strAskForCommand += 'd: Decrease force\n'
    
    strAskForCommand += '-->'

    return input(strAskForCommand)

def publisher(args=None):
    """Main loop which requests new commands and publish them on the Robotiq2FGripperRobotOutput topic."""
    rclpy.init(args=args)

    node = Node('Robotiq2FGripperSimpleController')
    
    pub = node.create_publisher(outputMsg, 'Robotiq2FGripperRobotOutput', 1)

    command = outputMsg()

    try:
        while rclpy.ok():

            command = genCommand(askForCommand(command), command)

            pub.publish(command)

            sleep(0.1)
    except KeyboardInterrupt:
        pass                

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    publisher()
