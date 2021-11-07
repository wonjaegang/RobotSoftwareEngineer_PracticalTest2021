# 가속운동하는 물체의 특성 출력

import matplotlib.pyplot as plt

# 물체 운동 데이터 셋
s0 = [0, 0, 0, 20, 30]
v0 = [0, 10, 20, 0, 0]
a0 = [5, 1, 0, 5, 2]
v_max = [10, 20, 20, 15, 10]
s1 = [100, 100, 100, 100, 100]
v1 = [0, 10, 20, 0, 0]
a1 = [-5, -1, 0, -10, -2]


# 반올림 함수
def round1(num):
    return round(num, 2)


if __name__ == "__main__":
    for i in range(5):
        # 상태 초기화
        print("%dth" % (i + 1))
        dt = 0.01
        s, v, a = [s0[i]], [v0[i]], [a0[i]]
        state = "acceleration"

        # 이동루프
        while state != "end":
            # 속도 계산
            v.append(v[-1] + dt * a[-1])
            print("v:", v[-1], end=', ')

            # 위치계산
            s.append(s[-1] + dt * v[-1])
            print("s:", s[-1])

            # 가속도 계산 및 상태 업데이트
            # 가속상태
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

            # 최대속도에서 등속운동
            elif state == "max velocity":
                if round1(2 * -a1[i] * (s1[i] - s[-1])) <= \
                        round1((v[-1] - v1[i]) * (v[-1] + v1[i])):
                    print("max -> dec")
                    state = "deceleration"
                    a.append(a1[i])
                else:
                    a.append(0)

            # 감속상태
            elif state == "deceleration":
                # s로 계산하면 오차가 누적되므로 v로 계산하는게 바람직하다.
                # 그러나 a1이 0인경우 v를 이용한 계산이 불가능하므로 s로 계산한다.
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
        plt.figure(1)
        plt.plot([dt * i for i in range(len(s))], s)
        plt.title("s-t Graph")
        plt.xlabel('t(sec)')
        plt.ylabel('s(m)')
        plt.legend([1, 2, 3, 4, 5])

        plt.figure(2)
        plt.plot([dt * i for i in range(len(v))], v)
        plt.title("v-t Graph")
        plt.xlabel('v(m/sec)')
        plt.ylabel('s(m)')
        plt.legend([1, 2, 3, 4, 5])

        plt.figure(3)
        plt.plot([dt * i for i in range(len(a))], a)
        plt.title("a-t Graph")
        plt.xlabel('t(sec)')
        plt.ylabel('a(m/s^2)')
        plt.legend([1, 2, 3, 4, 5])

    plt.show()
