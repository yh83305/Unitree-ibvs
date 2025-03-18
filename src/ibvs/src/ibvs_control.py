#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped
import sys
sys.path.append("./z1_sdk/lib")
import unitree_arm_interface
import time
np.set_printoptions(precision=3, suppress=True)

class VisualServo:
    def __init__(self):
        rospy.init_node('april_tag_visual_servo', anonymous=True)
        
        self.twist = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.arm = unitree_arm_interface.ArmInterface(hasGripper=True)
        self.arm.setFsmLowcmd()
        self.armModel = self.arm._ctrlComp.armModel

        # self.dt = self.arm._ctrlComp.dt
        self.go_to_Initialposition()
        self.q = self.arm.lowstate.getQ()
        self.dt = 0.05

        self.qd_min = -0.1  # 设定最小值
        self.qd_max = 0.1   # 设定最大值

        self.twist_sub = rospy.Subscriber('/arm/twist', TwistStamped, self.twist_callback)
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.publish_arm_command_qp)

        print("control node start.")

    def twist_callback(self, msg):
        self.twist = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z
        ])
        print(self.twist)

    def go_to_Initialposition(self):
        duration = 1000
        lastPos = self.arm.lowstate.getQ()
        targetPos = np.array([0.0, 1.5, -1.0, -0.54, 0.0, 0.0]) #forward

        for i in range(0, duration):
            self.arm.q = lastPos*(1-i/duration) + targetPos*(i/duration)# set position
            self.arm.qd = (targetPos-lastPos)/(duration*0.002) # set velocity
            self.arm.tau = self.armModel.inverseDynamics(self.arm.q, self.arm.qd, np.zeros(6), np.zeros(6)) # set torque
            self.arm.setArmCmd(self.arm.q, self.arm.qd, self.arm.tau)
            self.arm.sendRecv()
            time.sleep(self.arm._ctrlComp.dt)

        self.q = targetPos

    def publish_arm_command_inv(self, event):
        J = self.armModel.CalcJacobian(self.q)
        qd1 = np.linalg.pinv(J) @ self.twist
        qd2 = np.linalg.pinv(self.armModel.CalcJacobian(self.q + 0.5 * self.dt * qd1)) @ self.twist  # k2 = qd at (q + dt/2 * k1)
        qd3 = np.linalg.pinv(self.armModel.CalcJacobian(self.q + 0.5 * self.dt * qd2)) @ self.twist  # k3 = qd at (q + dt/2 * k2)
        qd4 = np.linalg.pinv(self.armModel.CalcJacobian(self.q + self.dt * qd3)) @ self.twist  # k4 = qd at (q + dt * k3)

        self.q += (self.dt / 6) * (qd1 + 2 * qd2 + 2 * qd3 + qd4)
        self.arm.q =  self.q
        self.arm.qd = np.clip(qd1, self.qd_min, self.qd_max)
        self.arm.tau = self.armModel.inverseDynamics(np.zeros(6), np.zeros(6), np.zeros(6), np.zeros(6)) 

        self.arm.setArmCmd(self.arm.q, self.arm.qd, self.arm.tau)
        self.arm.sendRecv()

    def publish_arm_command_qp(self, event):
        qd = self.armModel.solveQP(self.twist, self.q, self.dt)
        self.q += self.dt * qd
        self.arm.q =  self.q
        self.arm.qd = np.clip(qd, self.qd_min, self.qd_max)
        self.arm.tau = self.armModel.inverseDynamics(self.arm.q, self.arm.qd, np.zeros(6), np.zeros(6)) 

        self.arm.setArmCmd(self.arm.q, self.arm.qd, self.arm.tau)
        self.arm.sendRecv()

    def stop(self):
        print("Shutting down")
        self.arm.loopOn()
        self.arm.backToStart()
        self.arm.loopOff()

if __name__ == "__main__":
    servo = VisualServo()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        servo.stop()
