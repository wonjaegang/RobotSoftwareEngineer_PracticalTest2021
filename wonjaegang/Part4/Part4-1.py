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
a1 = [-5, -1, 0, -10, -2]


def round1(num):
    return round(num, 2)


if __name__ == "__main__":
    for i in range(5):
        print("%dth" % (i + 1))
        dt = 0.01
        s, v, a = [s0[i]], [v0[i]], [a0[i]]
        state = "acceleration"
        while state != "end":
            # 속도 계산
            v.append(v[-1] + dt * a[-1])
            if v[-1] >= v_max[i]:
                v[-1] = v_max[i]
            print("v:", v[-1], end=', ')

            # 위치계산
            s.append(s[-1] + dt * v[-1])
            print("s:", s[-1])

            # 가속도 계산 및 상태 업데이트
            if state == "acceleration":
                if round1(v[-1]) == round1(v_max[i]):
                    print("acc -> max")
                    state = "max velocity"
                    a.append(0)
                elif round1(2 * -a1[i] * (s1[i] - s[-1])) <= \
                        round1((v[-1] - v1[i]) * (v[-1] + v1[i])):
                    print("acc -> dec")
                    state = "deceleration"
                else:
                    a.append(a0[i])
            elif state == "max velocity":
                if round1(2 * -a1[i] * (s1[i] - s[-1])) <= \
                        round1((v[-1] - v1[i]) * (v[-1] + v1[i])):
                    print("max -> dec")
                    state = "deceleration"
                    a.append(a1[i])
                else:
                    a.append(0)
            elif state == "deceleration":
                if a1[i] == 0:
                    if round1(s[-1]) >= round1(s1[i]):
                        print("dec -> end")
                        state = "end"
                        a.append(a1[i])
                    else:
                        a.append(a1[i])
                else:
                    if round1(v[-1]) == round1(v1[i]):
                        print("dec -> end")
                        state = "end"
                        a.append(a1[i])
                    else:
                        a.append(a1[i])

        plt.plot([dt * i for i in range(len(s))], s)

        # plt.figure(2)
        # plt.plot([dt * i for i in range(len(s))], v)
        #
        # plt.figure(3)
        # plt.plot([dt * i for i in range(len(s))], a)

    plt.legend([1, 2, 3, 4, 5])
    plt.show()
