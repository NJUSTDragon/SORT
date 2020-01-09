# -*- coding: utf-8 -*-
import numpy as np
from kalman_tracker import KalmanTracker
from scipy.optimize import linear_sum_assignment


def boxes_iou(dets, pres):
    # top left xy
    top_l = np.maximum(dets[:, None, :2], pres[:, :2])
    # buttom right xy
    buttom_r = np.minimum(dets[:, None, 2:], pres[:, 2:])
    iou_area = np.prod(buttom_r - top_l, axis=2) * (top_l < buttom_r).all(axis=2)
    dets_area = np.prod(dets[:, 2:] - dets[:, :2], axis=1)
    pres_area = np.prod(pres[:, 2:] - pres[:, :2], axis=1)
    return iou_area / (dets_area[:, None] + pres_area - iou_area)


class SORT(object):
    def __init__(self, setIOUThreshould = 0.7, setMissThreshould = 10, setValidThreshould = 3):
        self._iouthreshould = setIOUThreshould
        self._missthreshould = setMissThreshould
        self._valdthreshould = setValidThreshould
        self.tracker_managers = list()

    def _assigment(self, current_dets, predict_boxes):
        matches, unmatched_detections, unmatched_trackers = list(), list(), list()
        if len(predict_boxes) == 0:
            unmatched_detections = list(range(len(current_dets)))
        elif len(current_dets) == 0:
            unmatched_trackers = list(range(len(predict_boxes)))
        else:
            iou = boxes_iou(current_dets, predict_boxes)
            row, col = linear_sum_assignment(-iou)
            for i in range(current_dets.shape[0]):
                if i not in row:
                    unmatched_detections.append(i)
            for j in range(predict_boxes.shape[0]):
                if j not in col:
                    unmatched_trackers.append(j)
            for i, j in zip(row, col):
                if iou[i, j] >= self._iouthreshould:
                    matches.append([i, j])
                else:
                    unmatched_detections.append(i)
                    unmatched_trackers.append(j)
        matches = np.array(matches)
        unmatched_detections = np.array(unmatched_detections)
        unmatched_trackers = np.array(unmatched_trackers)
        return matches, unmatched_detections, unmatched_trackers

    def trackers_update(self, dets, matches, unmatched_detections, unmatched_trackers):
        # 针对未与trackers匹配上的detections初始化新的Kalman追踪器
        for unmatched_detection in unmatched_detections:
            tracker = KalmanTracker(dets[unmatched_detection])
            self.tracker_managers.append(tracker)
        # 已与trackers匹配上的detections进行update
        for matched in matches:
            matched_detect_index, matched_tracker_index = matched
            self.tracker_managers[matched_tracker_index].update(dets[matched_detect_index])
        # 未与detections匹配上的trackers增加misscount,同时tracking_time置0
        for unmatched_tracker in unmatched_trackers:
            self.tracker_managers[unmatched_tracker].misscount += 1
            # # 策略1： 仅3帧连续跟踪成功才输出轨迹，即使中间断了一帧，也要重新开始计算连续3帧才会输出
            # self.tracker_managers[unmatched_tracker].tracking_time = 0

            # 策略2： 仅3帧连续跟踪成功才输出轨迹，即使中间断了帧，只要后面有帧匹配上则继续输出轨迹，因而增加tracking_flag标志
            self.tracker_managers[unmatched_tracker].tracking_flag = False  # 若trackers未能正确匹配，则说明轨迹中断，flag置0使轨迹不输出

    def track(self, dets, scores):
        predict_boxes = list()  # 存储每个tracker的predict位置，同时记录tracker序号
        for tracker in self.tracker_managers:
            predict_box = tracker.predict()
            predict_boxes.append(predict_box)
        predict_boxes = np.array(predict_boxes)
        matches, unmatched_detections, unmatched_trackers = self._assigment(dets, predict_boxes)
        self.trackers_update(dets, matches, unmatched_detections, unmatched_trackers)
        boxes, ids = list(), list()
        tracker_count = len(self.tracker_managers)
        for tracker in reversed(self.tracker_managers):
            tracker_count -= 1
            # # 策略1：仅3帧连续跟踪成功才输出保存轨迹，即使中间断了1帧，也要重新开始计算连续3帧才会输出
            # if tracker.misscount <= self._missthreshould and tracker.tracking_time > self._valdthreshould:

            # 策略2：仅3帧连续跟踪成功才输出保存轨迹， 即使中间断了1帧，只要后面有帧匹配上则继续输出轨迹
            if tracker.tracking_flag and tracker.tracking_time > self._valdthreshould:
                boxes.append(tracker.state()[:4, 0])
                ids.append(tracker.id)
            elif tracker.misscount > self._missthreshould:
                self.tracker_managers.pop(tracker_count)
        return boxes, ids


if __name__=='__main__':
    # 测试SORT
    MOT_tracker = SORT()






