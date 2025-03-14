import sys
sys.path.append("./z1_sdk/lib")
import unitree_arm_interface
import time
import numpy as np

np.set_printoptions(precision=5, suppress=True)
arm = unitree_arm_interface.ArmInterface(hasGripper=True)
armModel = arm._ctrlComp.armModel
arm.setFsmLowcmd()

# 运行时间（单位：循环次数）
duration = 1000  

target_twist = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.1])

# 初始关节角度
q = arm.lowstate.getQ()
dt = arm._ctrlComp.dt 


for i in range(duration):
    # 计算雅可比矩阵（6x6）
    J = armModel.CalcJacobian(q)  # 机械臂雅可比矩阵

    # 计算关节速度：qd = J⁻¹ * twist
    qd1 = np.linalg.pinv(J) @ target_twist  # k1 = qd
    qd2 = np.linalg.pinv(armModel.CalcJacobian(q + 0.5 * dt * qd1)) @ target_twist  # k2 = qd at (q + dt/2 * k1)
    qd3 = np.linalg.pinv(armModel.CalcJacobian(q + 0.5 * dt * qd2)) @ target_twist  # k3 = qd at (q + dt/2 * k2)
    qd4 = np.linalg.pinv(armModel.CalcJacobian(q + dt * qd3)) @ target_twist  # k4 = qd at (q + dt * k3)

    # RK4 计算 q 更新
    q += (dt / 6) * (qd1 + 2 * qd2 + 2 * qd3 + qd4)

    # 计算关节扭矩（可选）
    tau = armModel.inverseDynamics(np.zeros(6), np.zeros(6), np.zeros(6), np.zeros(6)) 

    # 设置控制指令
    arm.q = q  # 位置保持不变
    arm.qd = qd1  # 由末端Twist计算的关节速度
    arm.tau = tau  # 计算出的关节力矩

    # 发送指令
    arm.setArmCmd(arm.q, arm.qd, arm.tau)
    arm.sendRecv()

    time.sleep(dt)

# 停止控制循环
arm.loopOn()
arm.backToStart()
arm.loopOff()
