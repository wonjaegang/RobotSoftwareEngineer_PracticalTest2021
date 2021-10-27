# 1) 액추에이터의 각도에 따른 매니퓰레이터의 자세 출력 - 3D 매니퓰레이터


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


# 매니퓰레이터 기본 사양
l1, l2, l3, l4, l5, l6 = 4, 1, 2, 1, 1, 1


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


# 변환 행렬 함수
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


# 변환행렬들을 이용한 관절 및 공구단 위치 계산 함수
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


# 시간에 따른 각도 함수 계산
def motorAngle(w0, wf, tf, t):
    a0 = w0
    a1 = 0
    a2 = 3 / (tf ** 2) * (wf - w0)
    a3 = -2 / (tf ** 3) * (wf - w0)
    w = a0 + a1 * t + a2 * (t ** 2) + a3 * (t ** 3)
    return w


if __name__ == "__main__":
    # 매니퓰레이터 자세 출력 코드 - 3D:
    # plt 객체가 아닌 ax 객체를 사용해야한다. 때문에 여러 메소드가 달라지는 것을 조심하자!
    plt.figure(1)
    ax = plt.axes(projection='3d')

    plot1 = jointLocation(0, 0, 0, 0, 0)
    ax.plot(plot1[0], plot1[1], plot1[2], 'o-')

    plot2 = jointLocation(np.pi / 2, np.pi / 2, np.pi / 2, np.pi / 2, np.pi / 2)
    ax.plot(plot2[0], plot2[1], plot2[2], 'o-')

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.legend(["Before", "After"])
    ax.set_title("Manipulator pose at pi/2")

    # 시간에 따른 각도 함수 출력
    plt.figure(2)

    time = np.linspace(0, 10)
    plt.plot(time, motorAngle(0, np.pi / 2, 10, time))

    plt.xlabel('Time')
    plt.ylabel('Angle')
    plt.title("Actuator angle over time")

    # 시간에 따른 매니퓰레이터 자세 출력
    plt.figure(3)
    ax = plt.axes(projection='3d')

    time = np.linspace(0, 10)
    for i in time:
        plot3 = jointLocation(motorAngle(0, np.pi / 2, 10, i),
                              motorAngle(0, np.pi / 2, 10, i),
                              motorAngle(0, np.pi / 2, 10, i),
                              motorAngle(0, np.pi / 2, 10, i),
                              motorAngle(0, np.pi / 2, 10, i))
        ax.plot(plot3[0], plot3[1], plot3[2])

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_title("Manipultor pose over time")

    plt.show()
