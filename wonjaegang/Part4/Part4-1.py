# 가속운동하는 물체의 특성 출력

import numpy as np
import matplotlib.pyplot as plt
import decimal

# 물체 운동 데이터 셋
s0 = [0, 0, 0, 20, 30]
v0 = [0, 10, 20, 0, 0]
a0 = [5, 1, 0, 5, 2]
v_max = [10, 20, 20, 15, 10]
s1 = [100, 100, 100, 100, 100]
v1 = [0, 10, 20, 0, 0]
a1 = [5, 1, 0, 10, 2]


if __name__ == "__main__":

    # s-t 그래프 출력
    plt.figure(1)
    plt.axes().set_aspect('equal')

    for i in range(5):
        dt = 0.1
        s, v, a = [s0[i]], [v0[i]], [a0[i]]
        state = "acceleration"
        while state != "end":
            # 속도 계산
            v.append(v[-1] + dt * a[-1])
            print(v[-1], end=', ')

            # 위치계산
            s.append(s[-1] + dt * v[-1])
            print(s[-1])

            # 가속도 계산 및 상태 업데이트
            if state == "acceleration":
                if decimal.Decimal(v[-1]) == decimal.Decimal(v_max[i]):
                    print("acc -> max")
                    state = "max velocity"
                    a.append(0)
                else:
                    a.append(a0[i])
            elif state == "max velocity":
                if decimal.Decimal(s1[i] - s[-1]) == \
                        decimal.Decimal((v_max[i] - v1[i]) / a1[i] * (v_max[i] + v1[i]) / 2):
                    print("max -> dec")
                    state = "deceleration"
                    a.append(a1[i])
                else:
                    a.append(0)
            elif state == "deceleration":
                if s[-1] == s1[i]:
                    print("dec -> end")
                    state = "end"
                    a.append(a1[i])
                else:
                    a.append(a1[i])

        plt.plot([dt * i for i in range(len(s))], s)
        # plt.plot([dt * i for i in range(len(s))], v)
        # plt.plot([dt * i for i in range(len(s))], a)

    plt.show()
