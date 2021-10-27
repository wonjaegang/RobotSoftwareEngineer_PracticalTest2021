# 1) 액추에이터의 각도에 따른 매니퓰레이터의 자세출력

import numpy as np
import matplotlib.pyplot as plt


# 매니풀레이터 기본 사양
# 2차원 다관절 매니퓰레이터, 링크 초기위치 그림참고
l1 = 2
l2 = 1
l3 = 1
# 3차원 다관절 매니퓰레이터


# DH 파라미터
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


# PSO


if __name__ == "__main__":
    # 행렬 출력 포맷
    np.set_printoptions(formatter={'float_kind': lambda x: "{0:0.3f}".format(x)})
    print(DH_parameter(np.pi, 2, 1, np.pi / 2))
    pass
