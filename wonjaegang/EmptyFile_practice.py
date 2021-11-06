# PSO 3D 연습

import numpy as np
import matplotlib.pyplot as plt
import random

# 매니퓰레이터 기본 사양
l1, l2, l3, l4, l5, l6 = 4, 1, 2, 1, 1, 1


# DH parameter
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


# Homogeneous transformation
def T01(motor1):
    return DH_parameter(motor1, l1, 0, np.pi / 2)


def T12(motor2):
    return DH_parameter(motor2, -l2, l3, 0)


def T23(motor3):
    return DH_parameter(motor3, 0, l4, -np.pi / 2)


def T34(motor4):
    return DH_parameter(motor4 + np.pi / 2, 0, 0, np.pi / 2)


def T45(motor5):
    return DH_parameter(motor5, l5 + l6, 0, 0)


# Location of each joint
def jointLocation(m1, m2, m3, m4, m5):
    P0 = np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    P1 = T01(m1) @ P0
    P2 = T01(m1) @ T12(m2) @ P0
    P3 = T01(m1) @ T12(m2) @ T23(m3) @ P0
    P4 = T01(m1) @ T12(m2) @ T23(m3) @ T34(m4) @ P0
    P5 = T01(m1) @ T12(m2) @ T23(m3) @ T34(m4) @ T45(m5) @ P0
    X = [p[0, 3] for p in [P0, P1, P2, P3, P4, P5]]
    Y = [p[1, 3] for p in [P0, P1, P2, P3, P4, P5]]
    Z = [p[2, 3] for p in [P0, P1, P2, P3, P4, P5]]
    return X, Y, Z


# Inverse kinematics using PSO
def IK_PSO(T_target):

    plt.ion()
    figure = plt.subplots(figsize=(8, 6))[0]
    ax = plt.axes(projection='3d')

    def loss(d1, d2, d3, d4, d5):
        T_now = T01(d1) @ T12(d2) @ T23(d3) @ T34(d4) @ T45(d5)
        k = 0.5
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

    targetError = 0.00000001
    population = 50
    particleS = [[random.uniform(-np.pi / 2, np.pi / 2) for _ in range(5)] for _ in range(population)]
    particleV = [[random.uniform(-np.pi / 2, np.pi / 2) for _ in range(5)] for _ in range(population)]
    particleBest = [[random.uniform(-np.pi / 2, np.pi / 2) for _ in range(5)] for _ in range(population)]
    particleBestValue = [loss(*best) for best in particleBest]
    globalBest = [random.uniform(-np.pi / 2, np.pi / 2) for _ in range(5)]
    globalBestValue = loss(*globalBest)

    error = float('inf')
    count = 0
    while error > targetError:
        for i in range(population):
            w = 0.4
            c1 = 0.8
            r1 = random.uniform(0, 1)
            c2 = 0.8
            r2 = random.uniform(0, 1)
            particleV[i] = [w * vi + c1 * r1 * (pb - si) + c2 * r2 * (gb - si)
                            for si, vi, pb, gb in zip(particleS[i], particleV[i], particleBest[i], globalBest)]
            particleS[i] = [si + vi for si, vi in zip(particleS[i], particleV[i])]

            loss_now = loss(*particleS[i])
            if loss_now < particleBestValue[i]:
                particleBest[i] = particleS[i]
                particleBestValue[i] = loss_now
                if loss_now < globalBestValue:
                    globalBest = particleS[i]
                    globalBestValue = loss_now

            ax.plot(*jointLocation(*particleS[i]))

        error = globalBestValue
        count += 1
        print("No %d best loss: %.8f" % (count, error))


        figure.canvas.draw()
        figure.canvas.flush_events()
        plt.cla()

    return globalBest


if __name__ == "__main__":
    # plt.figure(1)
    # ax = plt.axes(projection='3d')
    #
    # ax.plot(*jointLocation(0, 0, 0, 0, 0))
    #
    T1 = np.array([[0, 0, 1, 3],
                   [0, -1, 0, 1],
                   [1, 0, 0, 6],
                   [0, 0, 0, 1]])
    T2 = np.array([[1, 0, 0, 2],
                   [0, 0, -1, -1],
                   [0, 1, 0, 5],
                   [0, 0, 0, 1]])
    IK_PSO(T1)
    # ax.plot(*jointLocation(*IK_PSO(T2)))
    #
    # ax.set_xlabel('x')
    # ax.set_ylabel('y')
    # ax.set_zlabel('z')
    # ax.set_title("Manipulator pose")
    #
    # plt.show()
