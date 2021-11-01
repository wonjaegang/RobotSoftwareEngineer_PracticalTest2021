# 2) End effector 의 위치에 따른 매니퓰레이터의 자세출력 - 2D

import numpy as np
import matplotlib.pyplot as plt
import random


# 매니퓰레이터 기본사양 (Part1-1 2D 그림을 그대로 사용)
l1, l2, l3 = 20, 10, 10


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


# DH 파라미터를 이용해 변환행렬함수 계산
def T01(motor1):
    return DH_parameter(motor1 + np.pi / 2, 0, l1, 0)


def T12(motor2):
    return DH_parameter(motor2 - np.pi / 2, 0, l2, 0)


def T23(motor3):
    return DH_parameter(motor3 - np.pi / 2, 0, l3, 0)


# 변환행렬을 이용해 관절 및 공구단의 위치 계산
def jointLocation(m1, m2, m3):
    P0 = [0, 0, 0, 1]
    P1 = T01(m1) @ P0
    P2 = T01(m1) @ T12(m2) @ P0
    P3 = T01(m1) @ T12(m2) @ T23(m3) @ P0
    X = [P0[0], P1[0], P2[0], P3[0]]
    Y = [P0[1], P1[1], P2[1], P3[1]]
    return X, Y


# 역기구학 계산: 목표변환행렬 [1, 0, 0, 10]
#                          [0, 1, 0, 30]
#                          [0, 0, 1, 0 ]
#                          [0, 0, 0, 1 ]
# 1) PSO 를 사용해보자
def PSO(matrixT):
    # PSO 초기화
    def loss(d1, d2, d3):
        T03 = T01(d1) @ T12(d2) @ T23(d3)
        k = 0.6
        unitP = np.array([[0],
                          [0],
                          [0],
                          [1]])
        unitR = np.array([[1, 0, 0],
                          [0, 1, 0],
                          [0, 0, 1],
                          [0, 0, 0]])
        Ep = (matrixT @ unitP - T03 @ unitP)[:3].__abs__().max() / (matrixT @ unitP)[:3].__abs__().max()
        Er = (matrixT @ unitR - T03 @ unitR)[:3].__abs__().max() / (matrixT @ unitR)[:3].__abs__().max()
        z = k * Ep + (1 - k) * Er
        # print(matrixT)
        # print(T03)
        # print((matrixT @ unitP - T03 @ unitP)[:3])
        # print((matrixT @ unitP - T03 @ unitP)[:3].__abs__().max())
        return z

    population = 30
    particleX = [[random.uniform(-np.pi / 2, np.pi / 2),
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

    bestLoss = float('inf')
    count = 0
    while bestLoss > 0.000001:
        for i in range(population):
            # 속도 계산
            w = 0.6
            c1 = 0.8
            r1 = random.uniform(0, 1)
            c2 = 0.8
            r2 = random.uniform(0, 1)
            particleV[i] = [w * vi + c1 * r1 * (pb - si) + c2 * r2 * (gb - si) for si, vi, pb, gb
                            in zip(particleX[i], particleV[i], particleBest[i], globalBest)]

            # 위치 계산
            particleX[i] = [si + vi for si, vi in zip(particleX[i], particleV[i])]

            # 평가 및 업데이트'
            lossNow = loss(*particleX[i])
            if lossNow < loss(*particleBest[i]):
                particleBest[i] = particleX[i]
                if lossNow < loss(*globalBest):
                    globalBest = particleX[i]
        bestLoss = loss(*globalBest)
        count += 1
        print("No.%d Best Loss : %0.10f" % (count, bestLoss))

    return particleX, globalBest


# 2). 역행렬 계산을 통해 구해보자.


if __name__ == "__main__":
    # 매니퓰레이터 자세 츌력
    plt.figure(1)
    plt.axes().set_aspect('equal')

    plot1 = jointLocation(0, 0, 0)
    plt.plot(plot1[0], plot1[1], 'o-')

    plot2 = jointLocation(np.pi / 4, np.pi / 4, np.pi / 4)
    plt.plot(plot2[0], plot2[1], 'o-')

    targetT = np.array([[1, 0, 0, 10],
                        [0, 1, 0, 30],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
    answers = PSO(targetT)
    print(answers)
    for answer in answers[0]:
        plot3 = jointLocation(*answer)
        plt.plot(plot3[0], plot3[1], 'o-')

    plot4 = jointLocation(*answers[1])
    plt.plot(plot4[0], plot4[1], 'o-')

    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Manipulator Pose')

    plt.show()
