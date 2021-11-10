# 2D 매니퓰레이터 역기구학

import numpy as np
import matplotlib.pyplot as plt
import random as rd

# 매니퓰레이터 기본 사양
l1, l2, l3 = 2, 1, 1


# DH 파라미터 함수
def DH_parameter(theta, d, a, alpha):
    T_theta = np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                        [np.sin(theta), np.cos(theta),  0, 0],
                        [0,             0,              1, 0],
                        [0,             0,              0, 1]])
    T_d = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, d],
                    [0, 0, 0, 1]])
    T_a = np.array([[1, 0, 0, a],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    T_alpha = np.array([[1, 0,              0,              0],
                        [0, np.cos(alpha),  -np.sin(alpha), 0],
                        [0, np.sin(alpha),  np.cos(alpha),  0],
                        [0, 0,              0,              1]])
    return T_theta @ T_d @ T_a @ T_alpha


# DH 파라미터를 이용한 동차변환행렬
def T01(motor1):
    return DH_parameter(motor1 + np.pi / 2, 0, l1, 0)


def T12(motor2):
    return DH_parameter(motor2 - np.pi / 2, 0, l2, 0)


def T23(motor3):
    return DH_parameter(motor3 - np.pi / 2, 0, l3, 0)


# 동차변환행렬을 이용한 관절 및 공구단 위치 계산 함수
def jointLocation(motor1, motor2, motor3):
    F0 = np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    F1 = T01(motor1) @ F0
    F2 = T01(motor1) @ T12(motor2) @ F0
    F3 = T01(motor1) @ T12(motor2) @ T23(motor3) @ F0
    X = [F[0, 3] for F in [F0, F1, F2, F3]]
    Y = [F[1, 3] for F in [F0, F1, F2, F3]]
    return X, Y


# PSO 를 이용한 역기구학 함수
def IK_PSO(T_target):
    # 손실함수 설정
    def loss(d1, d2, d3):
        k = 0.5
        T_now = T01(d1) @ T12(d2) @ T23(d3)
        unitP = np.array([[0],
                          [0],
                          [0],
                          [1]])
        unitR = np.array([[1, 0, 0],
                          [0, 1, 0],
                          [0, 0, 1],
                          [0, 0, 0]])
        Ep = (T_now @ unitP - T_target @ unitP)[:3].__abs__().max() / (T_target @ unitP)[:3].__abs__().max()
        Er = (T_now @ unitR - T_target @ unitR)[:3].__abs__().max() / (T_target @ unitR)[:3].__abs__().max()
        z = k * Ep + (1 - k) * Er
        return z
    # PSO 초기화
    dimension = 3
    targetError = 0.0000001
    population = 1000

    particleS = [[rd.uniform(-np.pi / 2, np.pi / 2) for _ in range(dimension)] for _ in range(population)]
    particleV = [[rd.uniform(-np.pi / 2, np.pi / 2) for _ in range(dimension)] for _ in range(population)]
    particlebest = [[rd.uniform(-np.pi / 2, np.pi / 2) for _ in range(dimension)] for _ in range(population)]
    particlebestValue = [loss(*best) for best in particlebest]
    globalbest = [rd.uniform(-np.pi / 2, np.pi / 2) for _ in range(dimension)]
    globalbestValue = loss(*globalbest)

    # 그래프 출력 설정
    plt.axes().set_aspect('equal')
    plt.ion()
    figure = plt.subplots(figsize=(8, 6))[0]

    # PSO 메인 루프
    error = float('inf')
    count = 0
    while error > targetError:
        # 각 파티클 마다
        for i in range(population):
            # 속도 업데이트
            w = 0.4
            c1 = 0.8
            r1 = rd.uniform(0, 1)
            c2 = 0.8
            r2 = rd.uniform(0, 1)
            particleV[i] = [w * vi + c1 * r1 * (pb - si) + c2 * r2 * (gb - si)
                            for si, vi, pb, gb in zip(particleS[i], particleV[i], particlebest[i], globalbest)]

            # 위치 업데이트
            particleS[i] = [si + vi for si, vi in zip(particleS[i], particleV[i])]

            # 최고점 업데이트
            loss_now = loss(*particleS[i])
            if loss_now < particlebestValue[i]:
                particlebest[i] = particleS[i]
                particlebestValue[i] = loss_now
                if loss_now < globalbestValue:
                    globalbest = particleS[i]
                    globalbestValue = loss_now

            # 자세 출력
            plt.plot(*jointLocation(*particleS[i]))

        # 평가
        count += 1
        error = globalbestValue
        print("No %d best loss: %.8f" % (count, error))

        # 자세출력 업데이트
        figure.canvas.draw()
        figure.canvas.flush_events()
        plt.cla()

    return globalbest


if __name__ == "__main__":
    # plt.figure(1)
    # plt.axes().set_aspect('equal')
    #
    # plt.plot(*jointLocation(0, 0, 0))

    T = np.array([[0, -1, 0, 1],
                  [1, 0, 0, 3],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    result = IK_PSO(T)
    # plt.plot(*jointLocation(*result))
    #
    # plt.title("Manipulator pose")
    # plt.xlabel("x")
    # plt.ylabel("y")
    # plt.show()
