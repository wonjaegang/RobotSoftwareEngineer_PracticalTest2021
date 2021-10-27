# 1) 액추에이터의 각도에 따른 매니퓰레이터의 자세출력

import numpy as np
import matplotlib.pyplot as plt


# 매니퓰레이터 기본 사양
# 2차워 다관절 매니퓰레이터, 링크 초기위치 그림참고
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


# 변환 행렬 계산
def T01(motor1):
    return DH_parameter(motor1 + np.pi / 2, 0, l1, 0)


def T12(motor2):
    return DH_parameter(motor2 - np.pi / 2, 0, l2, 0)


def T23(motor3):
    return DH_parameter(motor3 - np.pi / 2, 0, l3, 0)


# 변환 행렬을 통한 관절 및 공구단의 위치 계산
def jointLocation(m1, m2, m3):
    P0 = np.array([0, 0, 0, 1])
    X = [P0[0], (T01(m1) @ P0)[0], (T01(m1) @ T12(m2) @ P0)[0], (T01(m1) @ T12(m2) @ T23(m3) @ P0)[0]]
    Y = [P0[1], (T01(m1) @ P0)[1], (T01(m1) @ T12(m2) @ P0)[1], (T01(m1) @ T12(m2) @ T23(m3) @ P0)[1]]
    return X, Y


# 시간에 따른 각도함수 계산(Cubic Spline)
def motorAngle(w0, wf, tf, t):
    a0 = w0
    a1 = 0
    a2 = 3 / (tf ** 2) * (wf - w0)
    a3 = -2 / (tf ** 3) * (wf - w0)
    w = a0 + a1 * t + a2 * (t ** 2) + a3 * (t ** 3)
    return w


if __name__ == "__main__":
    # 매니퓰레이터 자세 출력 포맷
    np.set_printoptions(formatter={'float_kind': lambda x: "0:0.3f".format(x)})

    # 매니퓰레이터 자세 출력
    plt.figure(1)
    plt.axes().set_aspect('equal')

    plot1 = jointLocation(0, 0, 0)
    plt.plot(plot1[0], plot1[1])

    plot2 = jointLocation(np.pi / 6, np.pi / 6, np.pi / 6)
    plt.plot(plot2[0], plot2[1])

    plt.title("Manipulator pose at pi/6")
    plt.legend(["Before", "After"])

    # 시간에 따른 각도 출력(0 ~ pi/3)
    plt.figure(2)
    time = np.linspace(0, 10)
    plt.plot(time, motorAngle(0, np.pi / 3, 10, time))

    plt.title("Actuator angle over time")

    # 시간에 따른 매니퓰레이터 자세 출력
    plt.figure(3)
    time = np.linspace(0, 10, 20)
    for i in time:
        plot3 = jointLocation(motorAngle(0, np.pi / 3, 10, i),
                              motorAngle(0, np.pi / 3, 10, i),
                              motorAngle(0, np.pi / 3, 10, i))
        plt.plot(plot3[0], plot3[1])

    plt.title("Manipulator pose over time")

    plt.show()
