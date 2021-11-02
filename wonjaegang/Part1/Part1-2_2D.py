# 2) End effector 의 위치에 따른 매니퓰레이터의 자세출력 - 2D

import numpy as np
import matplotlib.pyplot as plt
import random

# 매니퓰레이터 기본 사양
l1, l2, l3 = 2, 1, 1


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


# DH 파라미터를 통한 변환행렬 함수
def T01(motor1):
    return DH_parameter(motor1 + np.pi / 2, 0, l1, 0)


def T12(motor2):
    return DH_parameter(motor2 - np.pi / 2, 0, l2, 0)


def T23(motor3):
    return DH_parameter(motor3 - np.pi / 2, 0, l3, 0)


# 변환행렬 함수를 통한 관절 및 공구단 위치 계산 함수
def jointLocation(m1, m2, m3):
    P0 = np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    P1 = T01(m1) @ P0
    P2 = T01(m1) @ T12(m2) @ P0
    P3 = T01(m1) @ T12(m2) @ T23(m3) @ P0
    X = [P0[0, 3], P1[0, 3], P2[0, 3], P3[0, 3]]
    Y = [P0[1, 3], P1[1, 3], P2[1, 3], P3[1, 3]]
    return X, Y


# PSO 를 통한 역기구학
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
    targetError = 0.00000001
    population = 100
    particleS = [[random.uniform(-np.pi / 2, np.pi / 2),
                  random.uniform(-np.pi / 2, np.pi / 2),
                  random.uniform(-np.pi / 2, np.pi / 2)] for _ in range(population)]
    particleV = [[random.uniform(-np.pi / 2, np.pi / 2),
                  random.uniform(-np.pi / 2, np.pi / 2),
                  random.uniform(-np.pi / 2, np.pi / 2)] for _ in range(population)]
    particleBest = [[random.uniform(-np.pi / 2, np.pi / 2),
                     random.uniform(-np.pi / 2, np.pi / 2),
                     random.uniform(-np.pi / 2, np.pi / 2)] for _ in range(population)]
    globalBest = [random.uniform(-np.pi / 2, np.pi / 2),
                  random.uniform(-np.pi / 2, np.pi / 2),
                  random.uniform(-np.pi / 2, np.pi / 2)]

    # PSO 루프
    count = 0
    error = float('inf')
    while error > targetError:
        for i in range(population):
            # 속도 업데이트
            w = 0.4
            c1 = 0.8
            r1 = random.uniform(0, 1)
            c2 = 0.8
            r2 = random.uniform(0, 1)
            particleV[i] = [w * vi + c1 * r1 * (pb - si) + c2 * r2 * (gb - si)
                            for si, vi, pb, gb in zip(particleS[i], particleV[i], particleBest[i], globalBest)]
            # 위치 업데이트
            particleS[i] = [si + vi for si, vi in zip(particleS[i], particleV[i])]

            # 평가 및 최고점 업데이트
            loss_now = loss(*particleS[i])
            if loss_now < loss(*particleBest[i]):
                particleBest[i] = particleS[i]
                if loss_now < loss(*globalBest):
                    globalBest = particleS[i]

        count += 1
        error = loss(*globalBest)
        print("NO.%d Best Loss: %.10f" % (count, error))

    return globalBest


if __name__ == "__main__":
    # 매니퓰레이터 초기 자세 출력
    plt.figure(1)
    plt.axes().set_aspect('equal')

    plt.plot(*jointLocation(0, 0, 0))

    plt.xlabel('x')
    plt.ylabel('y')
    plt.title("Manipulator initial pose")

    # 매니퓰레이터 역기구학 PSO 결과 출력
    plt.figure(2)
    plt.axes().set_aspect('equal')

    T = np.array([[1, 0, 0, 1],
                  [0, 1, 0, 2],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    result = IK_PSO(T)
    plt.plot(*jointLocation(*result))

    plt.xlabel('x')
    plt.ylabel('y')
    plt.title("Manipulator inverse kinematics by PSO")

    print(result)
    plt.show()
