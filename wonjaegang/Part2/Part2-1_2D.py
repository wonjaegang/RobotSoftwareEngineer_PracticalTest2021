# 2차원 매니퓰레이터의 동역학

import numpy as np
import matplotlib.pyplot as plt

# 매니퓰레이터 기본 사양
r = [[1, 1], [0.5, 1], [0.7, 0.3]]
m = [20, 20, 10]


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


# DH 파라미터를 이용한 동차변환행렬함수
def T01(motor1):
    return DH_parameter(motor1 + np.pi / 2, 0, sum(r[0]), 0)


def T12(motor2):
    return DH_parameter(motor2 - np.pi / 2, 0, sum(r[1]), 0)


def T23(motor3):
    return DH_parameter(motor3, 0, sum(r[2]), 0)


# 뉴턴 - 오일러 동역학
def NE_method():
    return 0


if __name__ == "__main__":
    # 각 링크들에 대해
    for i in range(len(m)):
        print(m[i])
