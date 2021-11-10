# 2D 매니퓰레이터 역기구학

import numpy as np
import matplotlib.pyplot as plt

# 매니퓰레이터 기본 사양
l1, l2, l3 = 2, 1, 1


# DH 파라미터 함수
def DH_parameter(theta, d, a, alpha):
    T_theta = np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                        [np.sin(theta), np.cos(theta),  0, 0],
                        [0,             0,              1, 0],
                        [0,             0,              0, 1]])
    T_d = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, d],
                    [0, 0, 0, 1]])
    T_a = np.array([[1, 0, 0, a],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    T_alpha = np.array([[1, 0,              0,              0],
                        [0, np.cos(alpha),  -np.sin(alpha), 0],
                        [0, np.sin(alpha),  np.cos(alpha),  0],
                        [0, 0,              0,              1]])
    return T_theta @ T_d @ T_a @ T_alpha


# DH 파라미터를 이용한 동차변환행렬
def T01(motor1):
    return DH_parameter(motor1 + np.pi / 2, 0, l1, 0)


def T12(motor2):
    return DH_parameter(motor2 - np.pi / 2, 0, l2, 0)


def T23(motor3):
    return DH_parameter(motor3 - np.pi / 2, 0, l3, 0)


# 동차변환행렬을 이용한 관절 및 공구단 위치 계산 함수
def jointLocation(motor1, motor2, motor3):
    F0 = np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    F1 = T01(motor1) @ F0
    F2 = T01(motor1) @ T12(motor2) @ F0
    F3 = T01(motor1) @ T12(motor2) @ T23(motor3) @ F0
    X = [F[0, 3] for F in [F0, F1, F2, F3]]
    Y = [F[1, 3] for F in [F0, F1, F2, F3]]
    return X, Y


if __name__ == "__main__":
    plt.figure(1)
    plt.axes().set_aspect('equal')

    plt.plot(*jointLocation(0, 0, 0))

    plt.title("Manipulator pose")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.show()
