# 기구학 ex-1

import numpy as np
import matplotlib.pyplot as plt
import random as rd

# 매니퓰레이터 기본 사양
l1, l2, l3 = 1, 1, 1


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


# DH 파라미터를 이용한 동차변환행렬
def T01(m1):
    return DH_parameter(m1, l1, 0, 0)


def T12(m2):
    return DH_parameter(np.pi / 2, l2 + m2, 0, np.pi / 2)


def T23(m3):
    return DH_parameter(0, l3 + m3, 0, 0)


# 동차변환행렬을 이용한 각 관절의 위치 함수
def jointLoacation(m1, m2, m3):
    F0 = np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    F1 = T01(m1) @ F0
    F2 = T01(m1) @ T12(m2) @ F0
    F3 = T01(m1) @ T12(m2) @ T23(m3) @ F0
    X = [F[0, 3] for F in [F0, F1, F2, F3]]
    Y = [F[1, 3] for F in [F0, F1, F2, F3]]
    Z = [F[2, 3] for F in [F0, F1, F2, F3]]
    return X, Y, Z


# PSO 를 활용한 역기구학 함수
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
        Ep = ((T_now - T_target) @ unitP)[:3].__abs__().max() / (T_target @ unitP)[:3].__abs__().max()
        Er = ((T_now - T_target) @ unitR)[:3].__abs__().max() / (T_target @ unitR)[:3].__abs__().max()
        z = k * Ep + (1 - k) * Er
        return z
    # PSO 초기화
    targetError = 0.0000001
    population = 2000
    particleS = [[rd.uniform(-np.pi / 2, np.pi / 2) for _ in range(3)] for _ in range(population)]
    particleV = [[rd.uniform(-np.pi / 2, np.pi / 2) for _ in range(3)] for _ in range(population)]
    particleBest = [[rd.uniform(-np.pi / 2, np.pi / 2) for _ in range(3)] for _ in range(population)]
    particleBestValue = [loss(*best) for best in particleBest]
    globalBest = [rd.uniform(-np.pi / 2, np.pi / 2) for _ in range(3)]
    globalBestValue = loss(*globalBest)

    # PSO 메인 루프
    error = float('inf')
    count = 0
    while error > targetError:
        # 파티클마다
        for i in range(population):
            # 속도계산
            w = 0.4
            c1 = 0.8
            r1 = rd.uniform(0, 1)
            c2 = 0.8
            r2 = rd.uniform(0, 1)
            particleV[i] = [w * vi + c1 * r1 * (pb - si) + c2 * r2 * (gb - si)
                            for si, vi, pb, gb in zip(particleS[i], particleV[i], particleBest[i], globalBest)]

            # 위치 계산
            particleS[i] = [si + vi for si, vi in zip(particleS[i], particleV[i])]

            # 최고점 업데이트
            loss_now = loss(*particleS[i])
            if loss_now < particleBestValue[i]:
                particleBest[i] = particleS[i]
                particleBestValue[i] = loss_now
                if loss_now < globalBestValue:
                    globalBest = particleS[i]
                    globalBestValue = loss_now

        # 루프 평가
        error = globalBestValue
        count += 1
        print("No %d best loss : %.8f" % (count, error))

    return globalBest


if __name__ == "__main__":
    plt.figure(1)
    ax = plt.axes(projection='3d')

    plot1 = jointLoacation(0, 0, 0)
    ax.plot(plot1[0], plot1[1], plot1[2], '-o')

    plot2 = jointLoacation(-np.pi / 3, 0, 0)
    ax.plot(plot2[0], plot1[1], plot2[2], '-o')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.figure(2)
    ax = plt.axes(projection='3d')

    plot1 = jointLoacation(0, 0, 0)
    ax.plot(plot1[0], plot1[1], plot1[2], '-o')

    T = np.array([[0, 0, 1, 2],
                  [1, 0, 0, 0],
                  [0, 1, 0, 3],
                  [0, 0, 0, 1]])
    result = IK_PSO(T)
    plot3 = jointLoacation(*result)
    ax.plot(plot3[0], plot3[1], plot3[2], '-o')

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.show()
