# 2) End effector 의 위치에 따른 매니퓰레이터의 자세출력 - 2D

import numpy as np
import matplotlib.pyplot as plt


# 매니퓰레이터 기본사양 (Part1-1 2D 그림을 그대로 사용)
l1, l2, l3 = 2, 1, 1


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


# DH 파라미터를 이용해 변환행렬함수 계산
def T01(motor1):
    return DH_parameter(motor1 + np.pi / 2, 0, l1, 0)


def T12(motor2):
    return DH_parameter(motor2 - np.pi / 2, 0, l2, 0)


def T23(motor3):
    return DH_parameter(motor3 - np.pi / 2, 0, l3, 0)


# 변환행렬을 이용해 좐절 및 공구단의 위치 계산
def jointLocation(m1, m2, m3):
    P0 = [0, 0, 0, 1]
    P1 = T01(m1) @ P0
    P2 = T01(m1) @ T12(m2) @ P0
    P3 = T01(m1) @ T12(m2) @ T23(m3) @ P0
    X = [P0[0], P1[0], P2[0], P3[0]]
    Y = [P0[1], P1[1], P2[1], P3[1]]
    return X, Y


if __name__ == "__main__":
    # 매니퓰레이터 자세 츌력
    plt.figure(1)
    plt.axes().set_aspect('equal')

    plot1 = jointLocation(0, 0, 0)
    plt.plot(plot1[0], plot1[1], 'o-')

    plot2 = jointLocation(np.pi / 4, np.pi / 4, np.pi / 4)
    plt.plot(plot2[0], plot2[1], 'o-')

    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Manipulator Pose')

    plt.show()
