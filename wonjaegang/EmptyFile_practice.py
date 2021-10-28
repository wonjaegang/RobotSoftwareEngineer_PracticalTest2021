# 액추에이터 각도에 따른 매니퓰레이터 자세 출력

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


# 매니퓰레이터 상제정보 (그림 참고)
l1, l2, l3, l4, l5, l6 = 4, 1, 2, 1.5, 1, 1


# DH 파라미터 함수
def DH_parameter(theta, d, a, alpha):
    T_theta = np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                        [np.sin(theta), np.cos(theta), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
    T_d = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, d],
                    [0, 0, 0, 1]])
    T_a = np.array([[1, 0, 0, a],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    T_alpha = np.array([[1, 0, 0, 0],
                        [0, np.cos(alpha), -np.sin(alpha), 0],
                        [0, np.sin(alpha), np.cos(alpha), 0],
                        [0, 0, 0, 1]])
    return T_theta @ T_d @ T_a @ T_alpha


# DH 파라미터를 통해 변환행렬 계산
def T01(motor1):
    return DH_parameter(motor1, l1, 0, np.pi / 2)


def T12(motor2):
    return DH_parameter(motor2, -l2, l3, 0)


def T23(motor3):
    return DH_parameter(motor3, 0, l4, -np.pi / 2)


def T34(motor4):
    return DH_parameter(motor4 + np.pi / 2, 0, 0, np.pi / 2)


def T45(motor5):
    return DH_parameter(motor5, l5 + l6, 0, 0)


# 변환행렬을 통한 관절 및 공구단의 위치 계산
def jointLocation(m1, m2, m3, m4, m5):
    P0 = [0, 0, 0, 1]
    P1 = T01(m1) @ P0
    P2 = T01(m1) @ T12(m2) @ P0
    P3 = T01(m1) @ T12(m2) @ T23(m3) @ P0
    P4 = T01(m1) @ T12(m2) @ T23(m3) @ T34(m4) @ P0
    P5 = T01(m1) @ T12(m2) @ T23(m3) @ T34(m4) @ T45(m5) @ P0
    X = [P0[0], P1[0], P2[0], P3[0], P4[0], P5[0]]
    Y = [P0[1], P1[1], P2[1], P3[1], P4[1], P5[1]]
    Z = [P0[2], P1[2], P2[2], P3[2], P4[2], P5[2]]
    return X, Y, Z


if __name__ == "__main__":
    # 매니퓰레이터 자세출력
    plt.figure(1)
    ax = plt.axes(projection='3d')

    plot1 = jointLocation(0, 0, 0, 0, 0)
    ax.plot(plot1[0], plot1[1], plot1[2], 'o-')

    plot2 = jointLocation(np.pi / 6, np.pi / 6, np.pi / 6, np.pi / 6, np.pi / 6)
    ax.plot(plot2[0], plot2[1], plot2[2], 'o-')

    ax.set_title("Manipulator Pose")
    ax.legend(["Before", "After"])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.show()
