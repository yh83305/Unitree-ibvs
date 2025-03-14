#!/usr/bin/env python3

import rospy
import cv2
import apriltag
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
np.set_printoptions(precision=3, suppress=True)

class AprilTag:
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

        self.k = -0.1

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

        self.bridge = CvBridge()

        options = apriltag.DetectorOptions(families="tag36h11")

        self.detector = apriltag.Detector(options)

        rospy.init_node('april_tag', anonymous=True)
        
        self.image_sub = rospy.Subscriber('/d435/color/image_raw', Image, self.image_callback)
        
        self.twist_pub = rospy.Publisher('/arm/twist', TwistStamped, queue_size=10)
        
        print("detect node start.")
        

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

        twist_msg = TwistStamped()
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.twist.linear.x = self.twist[0]
        twist_msg.twist.linear.y = self.twist[1]
        twist_msg.twist.linear.z = self.twist[2]
        twist_msg.twist.angular.x = self.twist[3]
        twist_msg.twist.angular.y = self.twist[4]
        twist_msg.twist.angular.z = self.twist[5]

        self.twist_pub.publish(twist_msg)


if __name__ == "__main__":
    visual = AprilTag()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        visual.stop()
