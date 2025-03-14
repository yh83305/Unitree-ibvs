#!/usr/bin/env python3
import rospy
import cv2
import apriltag
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import sys
sys.path.append("./z1_sdk/lib")
import unitree_arm_interface
import time
np.set_printoptions(precision=3, suppress=True)

class AprilTagVisualServo:
    def __init__(self):
        # 相机参数 (需要根据实际相机进行校准)
        self.fx = 462.1379699707031  # 焦距 x
        self.fy = 462.1379699707031  # 焦距 y
        self.cx = 320  # 主点 x
        self.cy = 240  # 主点 y
        
        # 期望 AprilTag 角点坐标 (单位: 像素)
        self.desired_corners = np.array([
            [270, 190], [370, 190], [370, 290], [270, 290]
        ], dtype=np.float32)

        self.jTcam = np.array([
                        [0,  0,  1, 0.050],
                        [-1,  0,  0, 0.033],
                        [0,  -1,  0, 0.062],
                        [0,  0,  0, 1   ]
                        ])
        
        self.camTj = self.inverse_pose_transform(self.jTcam)
        self.adjoint_jTcam = self.adjoint(self.jTcam)
        self.adjoint_camTj = self.adjoint(self.camTj)
        self.twist = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # 初始化 ROS
        rospy.init_node('april_tag_visual_servo', anonymous=True)
        
        # 订阅相机图像话题
        self.image_sub = rospy.Subscriber('/d435/color/image_raw', Image, self.image_callback)
        
        # 发布控制命令
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 初始化 AprilTag 检测器
        options = apriltag.DetectorOptions(families="tag36h11")

        # Create the detector with the specified options
        self.detector = apriltag.Detector(options)
        
        # 视觉伺服控制增益
        self.k = -0.1
        
        self.bridge = CvBridge()

        self.arm = unitree_arm_interface.ArmInterface(hasGripper=True)
        self.arm.setFsmLowcmd()
        self.q = self.arm.lowstate.getQ()
        self.armModel = self.arm._ctrlComp.armModel

        # self.dt = self.arm._ctrlComp.dt

        self.go_to_Initialposition()
        self.dt = 0.1
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.publish_arm_command)

        print("node start.")

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
        

    def compute_L(self, u, v, Z):
        x = (u - self.cx) / self.fx
        y = (v - self.cy) / self.fy
        L = np.zeros((2, 6))
        L[0, 0] = -1 / Z
        L[1, 0] = 0

        L[0, 1] = 0
        L[1, 1] = -1 / Z

        L[0, 2] = x / Z
        L[1, 2] = y / Z

        L[0, 3] = x * y
        L[1, 3] = 1 + y**2

        L[0, 4] = -(1 + x**2)
        L[1, 4] = -x * y

        L[0, 5] = y
        L[1, 5] = -x
        return L
    
    def skew_symmetric(self, vector):
        return np.array([
            [0, -vector[2], vector[1]],
            [vector[2], 0, -vector[0]],
            [-vector[1], vector[0], 0]
        ])

    def adjoint(self, T):
        R = T[:3, :3]
        t = T[:3, 3]
        
        t_skew = self.skew_symmetric(t)

        adj = np.zeros((6, 6))
        adj[:3, :3] = R
        adj[:3, 3:6] = t_skew @ R
        adj[3:6, 3:6] = R
        
        return adj
    
    def inverse_pose_transform(self, T):

        R = T[:3, :3]
        t = T[:3, 3]
        
        R_inv = R.T
        t_inv = -R_inv @ t
        
        T_inv = np.eye(4)
        T_inv[:3, :3] = R_inv
        T_inv[:3, 3] = t_inv
        
        return T_inv

    def detect_apriltag(self, color_image):
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(gray_image)
        
        if tags:
            for tag in tags:
                corners = np.int32(tag.corners)
                for i in range(4):
                    cv2.line(color_image, tuple(corners[i]), tuple(corners[(i+1) % 4]), (0, 255, 0), 2)
                
                for (i, (x, y)) in enumerate(corners):
                    cv2.circle(color_image, (x, y), 5, (0, 0, 255), -1)
                    cv2.putText(color_image, str(i), (x + 5, y - 5), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    
                for (ud, vd) in self.desired_corners:
                            cv2.circle(color_image, (int(ud), int(vd)), 5, (255, 0, 255), -1)
                            cv2.putText(color_image, str(i), (x + 5, y - 5), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    
                cv2.imshow("AprilTag Detection", color_image)
                cv2.waitKey(1)
                
                Z = 1.0
                errors = []
                L_stack = []
                
                for (i, (u, v)) in enumerate(corners):
                    ud, vd = self.desired_corners[i]
                    e = np.array([ (u - ud) / self.fx, (v - vd)/self.fy])
                    errors.append(e)
                    L_stack.append(self.compute_L(u, v, Z))
                
                errors = np.hstack(errors).reshape(-1, 1)
                L_stack = np.vstack(L_stack)
                L_inv = np.linalg.pinv(L_stack)
                twist_cam_vw = self.k * np.dot(L_inv, errors)
                twist_joint_vw = self.adjoint_jTcam @ twist_cam_vw
                twist_joint_wv = np.vstack([twist_joint_vw[3:], twist_joint_vw[:3]])
                
                return twist_joint_wv.flatten()
        
        else:
            cv2.imshow("AprilTag Detection", color_image)
            cv2.waitKey(1)
        
        return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def image_callback(self, msg):
        color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.twist = self.detect_apriltag(color_image)

    def publish_arm_command(self, event):
        J = self.armModel.CalcJacobian(self.q)
        qd1 = np.linalg.pinv(J) @ self.twist
        qd2 = np.linalg.pinv(self.armModel.CalcJacobian(self.q + 0.5 * self.dt * qd1)) @ self.twist  # k2 = qd at (q + dt/2 * k1)
        qd3 = np.linalg.pinv(self.armModel.CalcJacobian(self.q + 0.5 * self.dt * qd2)) @ self.twist  # k3 = qd at (q + dt/2 * k2)
        qd4 = np.linalg.pinv(self.armModel.CalcJacobian(self.q + self.dt * qd3)) @ self.twist  # k4 = qd at (q + dt * k3)

        self.q += (self.dt / 6) * (qd1 + 2 * qd2 + 2 * qd3 + qd4)
        tau = self.armModel.inverseDynamics(np.zeros(6), np.zeros(6), np.zeros(6), np.zeros(6)) 
        self.arm.q = self.q
        self.arm.qd = qd1
        self.arm.tau = tau

        print(self.arm.qd)

        self.arm.setArmCmd(self.arm.q, self.arm.qd, self.arm.tau)
        self.arm.sendRecv()

    def stop(self):
        self.arm.backToStart()
        self.arm.loopOff()
        cv2.destroyAllWindows()
        print("Shutting down")

if __name__ == "__main__":
    visual_servo = AprilTagVisualServo()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        visual_servo.stop()
