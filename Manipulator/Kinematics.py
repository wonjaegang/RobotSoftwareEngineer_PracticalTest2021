# 2차원 링크의 기구학

import numpy as np
import matplotlib.pyplot as plt

# Plot style
plt.rcParams['figure.figsize'] = (10, 5)


if __name__ == "__main__":
    theta1 = np.linspace(0, 2 * np.pi, 10)
    theta2 = np.linspace(0, 2 * np.pi, 10)

    x0 = 0
    y0 = 0

    l1 = 10
    x1 = l1 * np.cos(theta1)
    y1 = l1 * np.sin(theta1)

    l2 = 5
    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)

    # 플롯 준비
    fig = plt.figure()

    # 링크의 끝 점들을 플롯
    p1 = fig.add_subplot(1, 2, 1)
    p1.plot(x0, y0)
    p1.plot(x1, y1)
    p1.plot(x2, y2)

    # # 링크의 움직임을 플롯
    # p2 = fig.add_subplot(1, 2, 2)
    # for i in theta1:
    #     plt.plot([[x0, y0], [x1, y1]])

    plt.show()
