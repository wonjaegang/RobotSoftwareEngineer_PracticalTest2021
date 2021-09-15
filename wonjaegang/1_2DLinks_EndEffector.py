# 1. 2D Links end effector - Joint space
#     두 개의 2차원 링크로 이루어진 매니퓰레이터가 있다.
#     링크의 길이는 각각 10, 5 이며 두 액추에이터와 공구단의 초기 위치는 각각 (0, 0), (10, 0), (15, 0)이다.
#     1) 두 액추에이터가 같은 각속도로 한 바퀴 회전할 때, 각 관절과 공구단의 위치를 그래프로 나타내어라.
#     2) 각 액추에이터를 적당한 각속도로 움직여 10초 후의 공구단이 (0, 10)에 위치하게 하려한다. Cubic interpolation 을 사용해
#         액추에이터의 속도를 계획했을 때, 각 관절과 공구단의 위치를 그래프로 나타내어라.

import numpy as np
import matplotlib.pyplot as plt

# Plot style
plt.rcParams['figure.figsize'] = (5, 5)


def problem1():
    theta1 = np.linspace(0, 2 * np.pi, 100)
    theta2 = np.linspace(0, 2 * np.pi, 100)

    x0 = 0
    y0 = 0

    l1 = 10
    x1 = l1 * np.cos(theta1)
    y1 = l1 * np.sin(theta1)

    l2 = 5
    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)

    # 플롯 준비
    fig1 = plt.figure()

    # 링크의 끝 점들을 플롯
    plt.plot(x0, y0)
    plt.plot(x1, y1)
    plt.plot(x2, y2)


def problem2():
    # 역기구학 해를 먼저 구한다

    t = np.linspace(0, 10, 100)

    theta1 = np.linspace(0, 2 * np.pi, 100)
    theta2 = np.linspace(0, 2 * np.pi, 100)

    x0 = 0
    y0 = 0

    l1 = 10
    x1 = x0 + l1 * np.cos(theta1)
    y1 = y0 + l1 * np.sin(theta1)

    l2 = 5
    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)

    # 플롯 준비
    fig2 = plt.figure()

    # 링크의 끝 점들을 플롯
    plt.plot(x1, y1)
    plt.plot(x2, y2)


if __name__ == "__main__":
    problem1()
    problem2()
    plt.show()
