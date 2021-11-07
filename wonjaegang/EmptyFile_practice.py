# 가속운동하는 물체의 특성 출력

import matplotlib.pyplot as plt

# 물체 특성 데이터시트
s0 = [0, 0, 0, 20, 30]
v0 = [0, 10, 20, 0, 0]
a0 = [5, 1, 0, 5, 2]
v_max = [10, 20, 20, 15, 10]
s1 = [100, 100, 100, 100, 100]
v1 = [0, 10, 20, 0, 0]
a1 = [-5, -1, 0, -10, -2]


def round1(num):
    return round(num, 1)


if __name__ == "__main__":
    # 각 물체마다
    for i in range(5):
        # 물체 특성 초기화
        dt = 0.1
        s, v, a = [s0[i]], [v0[i]], [a0[i]]
        state = "acceleration"

        # 메인 루프
        while state != "end":
            # 속도 업데이트
            v.append(v[-1] + a[-1] * dt)

            # 위치 업데이트
            s.append(s[-1] + v[-1] * dt)

            # 가속도 및 상태 업데이트
            # 가속 상태
            if state == "acceleration":
                # 가속 -> 감속
                if round1(2 * a1[i] * (s1[i] - s[-1])) >= round1((v1[i] - v[-1]) * (v1[i] + v[-1])):
                    state = "deceleration"
                    a.append(a1[i])
                # 가속 -> 최고속도
                elif round1(v[-1]) == round1(v_max[i]):
                    state = "max velocity"
                    a.append(0)
                # 가속 -> 가속
                else:
                    a.append(a0[i])

            # 최고속도
            if state == "max velocity":
                # 최고속도 -> 감속
                if round1(2 * a1[i] * (s1[i] - s[-1])) >= round1((v1[i] - v[-1]) * (v1[i] + v[-1])):
                    state = "deceleration"
                    a.append(a1[i])
                # 최고속도 -> 최고속도
                else:
                    a.append(0)
            # 감속 상태
            if state == "deceleration":
                # 감속 -> 도착
                if round1(s[-1]) >= round1(s1[i]):
                    state = "end"
                    a.append(a1[i])
                # 감속 -> 감속
                else:
                    a.append(a1[i])

        plt.figure(1)
        plt.plot([dt * i for i in range(len(s))], s)

        plt.figure(2)
        plt.plot([dt * i for i in range(len(v))], v)

        plt.figure(3)
        plt.plot([dt * i for i in range(len(a))], a)

    plt.show()
