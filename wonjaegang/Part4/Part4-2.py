# Part 4-2 속도프로파일링

import numpy as np
import matplotlib.pyplot as plt

# 물체 데이터 시트
a = [10, 20, 20, 10, 30]
v_max = [50, 50, 70, 50, 80]
s = [100, 100, 100, 200, 200]


# 가속운동
def acceleration_V(t, accel):
    return t * accel


# 등속운동
def constant_v(v):
    return v


# 감속운동
def deceleration(t, decel, vo):
    return vo - decel * t


if __name__ == "__main__":
    dt = 0.1

    # 각 물체마다
    for i in range(len(a)):
        state = "acceleration"

        while state != "end":
            # 최고속도 도달 X
            if (a[i] * s[i]) ** 0.5 < v_max[i]:
                pass

            # 최고속도 도달 O
            else:
                pass
