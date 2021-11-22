# 기구학 ex-1:

import numpy as np
import matplotlib.pyplot as plt

# 매니퓰레이터 기본 사양
l1, l2, l3 = 1, 1, 1


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


# DH 파라미터를 이용한 동차변환행렬
def T01(m1):
    return DH_parameter(m1, l1, 0, 0)


def T12(m2):
    return DH_parameter(np.pi / 2, l2 + m2, 0, np.pi / 2)


def T23(m3):
    return DH_parameter(0, l3 + m3, 0, 0)


# 동차변환행렬을 이용한 각 관절의 위치 함수
def jointLoacation(m1, m2, m3):
    F0 = np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    F1 = T01(m1) @ F0
    F2 = T01(m1) @ T12(m2) @ F0
    F3 = T01(m1) @ T12(m2) @ T23(m3) @ F0
    X = [F[0, 3] for F in [F0, F1, F2, F3]]
    Y = [F[1, 3] for F in [F0, F1, F2, F3]]
    Z = [F[2, 3] for F in [F0, F1, F2, F3]]
    return X, Y, Z


if __name__ == "__main__":
    plt.figure(1)
    ax = plt.axes(projection='3d')

    plot1 = jointLoacation(0, 0, 0)
    print(plot1)
    ax.plot(plot1[0], plot1[1], plot1[2], '-o')

    plot1 = jointLoacation(-np.pi / 4, 1, 1)
    print(plot1)
    ax.plot(plot1[0], plot1[1], plot1[2], '-o')

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()
