# 1) 액추에이터의 각도에 따른 매니퓰레이터의 자세출력

import numpy as np
import matplotlib.pyplot as plt


# 매니풀레이터 기본 사양
# 2차원 다관절 매니퓰레이터, 링크 초기위치 그림참고
l1, l2, l3 = 2, 1, 1
# 3차원 다관절 매니퓰레이터


# DH 파라미터
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


# 기구학 계산
def T01(motor1):
    return DH_parameter(motor1 + np.pi / 2, 0, l1, 0)


def T12(motor2):
    return DH_parameter(motor2 - np.pi / 2, 0, l2, 0)


def T23(motor3):
    return DH_parameter(motor3 - np.pi / 2, 0, l3, 0)


def jointLocation(m1, m2, m3):
    P0 = np.array([0, 0, 0, 1])
    x = [P0[0], (T01(m1) @ P0)[0], (T01(m1) @ T12(m2) @ P0)[0], (T01(m1) @ T12(m2) @ T23(m3) @ P0)[0]]
    y = [P0[1], (T01(m1) @ P0)[1], (T01(m1) @ T12(m2) @ P0)[1], (T01(m1) @ T12(m2) @ T23(m3) @ P0)[1]]
    return x, y


if __name__ == "__main__":
    # 행렬 출력 포맷
    np.set_printoptions(formatter={'float_kind': lambda x: "{0:0.3f}".format(x)})

    # 매니퓰레이터 자세 출력
    plt.axes().set_aspect('equal')

    plot1 = jointLocation(0, 0, 0)
    plt.plot(plot1[0], plot1[1])

    plot2 = jointLocation(np.pi / 6, np.pi / 6, np.pi / 6)
    plt.plot(plot2[0], plot2[1])

    plt.legend(["Before", "After"])
    plt.show()
