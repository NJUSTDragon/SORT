# -*- coding: utf-8 -*-
import numpy as np
from filterpy.kalman import KalmanFilter


class KalmanTracker(object):
    # trackers初始计数器
    count = 1

    def __init__(self, box):
        self.kf = KalmanFilter(dim_x=8, dim_z=8)
        self.kf.F = np.array([[1, 0, 0, 0, 1, 0, 0, 0],
                              [0, 1, 0, 0, 0, 1, 0, 0],
                              [0, 0, 1, 0, 0, 0, 1, 0],
                              [0, 0, 0, 1, 0, 0, 0, 1],
                              [0, 0, 0, 0, 1, 0, 0, 0],
                              [0, 0, 0, 0, 0, 1, 0, 0],
                              [0, 0, 0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 0, 0, 0, 1]])
        self.kf.H = np.array([[1, 0, 0, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0, 0, 0],
                              [0, 0, 1, 0, 0, 0, 0, 0],
                              [0, 0, 0, 1, 0, 0, 0, 0],
                              [0, 0, 0, 0, 1, 0, 0, 0],
                              [0, 0, 0, 0, 0, 1, 0, 0],
                              [0, 0, 0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 0, 0, 0, 1]])
        self.kf.P *= 10.
        self.kf.P[4:, 4:] *= 100.
        self.kf.Q[4:, 4:] *= 0.01
        self.kf.Q[6:,6:] *= 0.01
        self.kf.R *= 10.  # 观测值的方差记为10个像素
        self.kf.x[:4, 0] = box.T
        self.v = list()  # 记录速度列表
        self.last_position = np.empty(4)  # 位置信息
        self.id = KalmanTracker.count
        KalmanTracker.count += 1
        self.misscount = 0
        self.tracking_time = 0  # 正确跟踪计时
        self.tracking_flag = False  # 正确跟踪标志

    def predict(self):
        self.last_position = self.state()[:4, 0]
        self.kf.predict()
        return self.kf.x_prior[:4, 0].tolist()

    def update(self, box):
        self.v.append((box - self.last_position).tolist())  # 计算速度 ,v是个列表
        data = np.hstack((box, self.v[-1]))
        self.kf.update(data)
        self.misscount = 0  # 匹配成功进行update后，丢失帧标志置0
        self.tracking_time += 1  # 匹配成功正确跟踪计时+1
        self.tracking_flag = True  # 匹配成功正确跟踪标志置1

    def state(self):
        return self.kf.x


if __name__=='__main__':
    # 测试追踪器
    tracker = KalmanTracker(np.array([1, 2, 3, 4]))
    print(f"1:\n{tracker.state()}")
    tracker.predict()
    print(f"2:\n{tracker.state()}")
    tracker.update(np.array([2, 3 , 4, 5]))
    print(f"3:\n{tracker.state()}")
    tracker.predict()
    print(f"4:\n{tracker.state()}")






