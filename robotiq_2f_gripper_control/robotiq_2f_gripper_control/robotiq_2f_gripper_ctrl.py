#!/usr/bin/env python3

import numpy as np

import rclpy
import time
from rclpy.node import Node
from robotiq_2f_gripper_control.msg import Robotiq2FGripperOutput as outputMsg
from robotiq_2f_gripper_control.msg import Robotiq2FGripperInput as inputMsg

class RobotiqCGripper(Node):
    def __init__(self):
        super().__init__('robotiq_2f_gripper_ctrl_test')
        self.cur_status = None
        self.status_sub = self.create_subscription(inputMsg, 'Robotiq2FGripperRobotInput', self._status_cb, 1)
        self.cmd_pub = self.create_publisher(outputMsg, 'Robotiq2FGripperRobotOutput', 1)

    def _status_cb(self, msg):
        self.cur_status = msg

    def wait_for_connection(self, timeout=-1):
        time.sleep(0.1)
        r = self.create_rate(30)
        start_time = time.time()
        while rclpy.ok():
            if (timeout >= 0. and time.time() - start_time > timeout):
                return False
            if self.cur_status is not None:
                return True
            r.sleep()
        return False

    def is_ready(self):
        return self.cur_status.g_sta == 3 and self.cur_status.g_act == 1

    def is_reset(self):
        return self.cur_status.g_sta == 0 or self.cur_status.g_act == 0

    def is_moving(self):
        return self.cur_status.g_gto == 1 and self.cur_status.g_obj == 0

    def is_stopped(self):
        return self.cur_status.g_obj != 0

    def object_detected(self):
        return self.cur_status.g_obj == 1 or self.cur_status.g_obj == 2

    def get_fault_status(self):
        return self.cur_status.g_flt

    def get_pos(self):
        po = self.cur_status.g_po
        return np.clip(0.087/(13.-230.)*(po-230.), 0, 0.087)

    def get_req_pos(self):
        pr = self.cur_status.gPR
        return np.clip(0.087/(13.-230.)*(pr-230.), 0, 0.087)

    def is_closed(self):
        return self.cur_status.g_po >= 230

    def is_opened(self):
        return self.cur_status.g_po <= 13

    # in mA
    def get_current(self):
        return self.cur_status.gCU * 0.1

    # if timeout is negative, wait forever
    def wait_until_stopped(self, timeout=-1):
        r = self.create_rate(30)
        start_time = time.time()
        while rclpy.ok():
            if (timeout >= 0. and time.time() - start_time > timeout) or self.is_reset():
                return False
            if self.is_stopped():
                return True
            r.sleep()
        return False

    def wait_until_moving(self, timeout=-1):
        r = self.create_rate(30)
        start_time = time.time()
        while rclpy.ok():
            if (timeout >= 0. and time.time() - start_time > timeout) or self.is_reset():
                return False
            if not self.is_stopped():
                return True
            r.sleep()
        return False

    def reset(self):
        cmd = outputMsg()
        cmd.r_act = 0
        self.cmd_pub.publish(cmd)

    def activate(self, timeout=-1):
        cmd = outputMsg()
        cmd.r_act = 1
        cmd.r_gto = 1
        cmd.r_pr = 0
        cmd.r_sp = 255
        cmd.r_fr = 150
        self.cmd_pub.publish(cmd)
        r = self.create_rate(30)
        start_time = time.time()
        while rclpy.ok():
            if timeout >= 0. and time.time() - start_time > timeout:
                return False
            if self.is_ready():
                return True
            r.sleep()
        return False

    def auto_release(self):
        cmd = outputMsg()
        cmd.r_act = 1
        cmd.r_atr = 1
        self.cmd_pub.publish(cmd)

    ##
    # Goto position with desired force and velocity
    # @param pos Gripper width in meters. [0, 0.087]
    # @param vel Gripper speed in m/s. [0.013, 0.100]
    # @param force Gripper force in N. [30, 100] (not precise)
    def goto(self, pos, vel, force, block=False, timeout=-1):
        cmd = outputMsg()
        cmd.r_act = 1
        cmd.r_gto = 1
        cmd.r_pr = int(np.clip((13.-230.)/0.087 * pos + 230., 0, 255))
        cmd.r_sp = int(np.clip(255./(0.1-0.013) * (vel-0.013), 0, 255))
        cmd.r_fr = int(np.clip(255./(100.-30.) * (force-30.), 0, 255))
        self.cmd_pub.publish(cmd)
        time.sleep(0.1)
        if block:
            if not self.wait_until_moving(timeout):
                return False
            return self.wait_until_stopped(timeout)
        return True

    def stop(self, block=False, timeout=-1):
        cmd = outputMsg()
        cmd.r_act = 1
        cmd.r_gto = 0
        self.cmd_pub.publish(cmd)
        time.sleep(0.1)
        if block:
            return self.wait_until_stopped(timeout)
        return True

    def open(self, vel=0.1, force=100, block=False, timeout=-1):
        if self.is_opened():
            return True
        return self.goto(1.0, vel, force, block=block, timeout=timeout)

    def close(self, vel=0.1, force=100, block=False, timeout=-1):
        if self.is_closed():
            return True
        return self.goto(-1.0, vel, force, block=block, timeout=timeout)

def main(args=None):
    rclpy.init(args=args)
    # 
    gripper = RobotiqCGripper()
    gripper.wait_for_connection()
    if gripper.is_reset():
        gripper.reset()
        gripper.activate()
    print(gripper.close(block=True))
    while rclpy.ok():
        print(gripper.open(block=False))
        time.sleep(0.11)
        gripper.stop()
        print(gripper.close(block=False))
        time.sleep(0.1)
        gripper.stop()

if __name__ == '__main__':
    main()
