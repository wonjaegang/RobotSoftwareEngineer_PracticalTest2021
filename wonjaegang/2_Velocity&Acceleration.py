# 2. Velocity and Acceleration - 기출
#     x 축 위의 원점으로부터 20m만큼 떨어진 곳에 정점이 있다. 정점은 가속도 5m/s^2, 최대속도 20m/s로 오른쪽으로 이동하며 원점으로부터
#     오른쪽으로 400m 떨어진 곳에 도달한다. 도달 전에 정점은 -5m/s^2으로 가속하며 속도를 줄이다가 도달지점에서 속도는 0이 된다. 정점의
#     거리, 속도, 가속도 그래프를 그리시오.

import numpy as np
import matplotlib.pyplot as plt

plt.rcParams['figure.figsize'] = (5, 10)

if __name__ == "__main__":
    # 등가속도 운동의 그래프
    def plotGraph(t1, t2, a0, v0, s0):
        t = np.linspace(t1, t2)
        a = a0 + (t - t1) * 0
        v = v0 + (t - t1) * a
        s = s0 + (t - t1) * v0 + ((t - t1) ** 2) * a * 0.5
        p1.plot(t, s, 'b')
        p2.plot(t, v, 'r')
        p3.plot(t, a, 'g')

    # 플롯 스타일
    fig = plt.figure()
    p1 = fig.add_subplot(3, 1, 1)
    p2 = fig.add_subplot(3, 1, 2)
    p3 = fig.add_subplot(3, 1, 3)
    p1.set_xlabel("Distance")
    p1.set_ylabel("Time")
    p2.set_xlabel("Velocity")
    p2.set_ylabel("Time")
    p3.set_xlabel("Acceleration")
    p3.set_ylabel("Time")

    # 가속 구간
    plotGraph(0, 4, 5, 0, 20)

    # 등속 구간
    plotGraph(4, 19, 0, 20, 60)

    # 감속 구간'
    plotGraph(19, 23, -5, 20, 360)

    plt.show()
