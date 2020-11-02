#### SORT（Simple Online and Realtime Tracking）

> sort算法的本质上是利用多个kalman滤波器进行目标跟踪，基于匈牙利匹配算法实现数据关联

+ 原始的SORT算法采用7个状态量$[u\;,v\;, s\;, r\;, \vec{u} \;,\vec{v} \;,\vec{s}]$  ,其中$[u\;,v\;,s\;, r]$ 利用检测结果作为观测量输入Kalman滤波器

+ 改进后的SORT算法采用8个状态量$[u\;,v \;, w \;, h \;, \vec{u} \;,\vec{v} \;,\vec{w} \;,\vec{h}]$  ,其中 $[u\;,v \;, w \;, h \;, \vec{u} \;,\vec{v} \;,\vec{w} \;,\vec{h}]$ 利用检测结果和追踪结果作为观测量输入Kalman滤波器

  > 速度状态量只能在当前tracker追踪的第3帧作为输入观测量，初始化Kalman时，速度初值给定为0。

+ 基本实现流程

  1. 针对第一帧的检测结果dets初始化trackers个kalman追踪器

  2. 针对每一个trackers，基于恒速CV线性模型实现先验位置预测

  3. 针对每一帧的检测结果dets与**当前trackers的先验位置**，利用匈牙利匹配算法实现数据关联

     > 数据关联结果有以下4种情况

     1. trackers与dets匹配成功：利用匹配成功的dets对相应的trackers进行update

     2. dets没有与合适的trackers匹配：对没有匹配成功的dets重新分配新的Kalman追踪器

     3. trackers没有与合适的dets匹配，这对应于2种情况：

        + 目标已经离开
        + 检测结果出现偏差（遮挡等原因导致）
     4. dets没有合适的trackers匹配，同时，trackers也没有合适的dets匹配：这将导致ID的切换

  4. 针对上述4种匹配结果，sort的解决策略：

     1. 利用Kalman算法update，得到新的修正位置（结合了先验位置和观测位置的修正值）

     2. 重新分配Kalman的dets开始新的轨迹，获得新的ID，**也导致ID的切换**

     3. 对于trackers没有与合适的dets匹配的，直接利用先验预测位置作为当前帧的追踪位置，不进行update，若超过阈值**T**(此处设置为1)未匹配到合适的dets，则pop当前trackers结束跟踪

        > 原始的sort直接利用检测的位置信息作为观测量并结合CV线性模型预测位置准确率较低，因而加入速度信息作为观测量可以适当放大阈值**T**

+ 加入速度信息作为观测量还可以在一定程度上避免对向相遇的行人的ID切换问题，但是对于长期遮挡引起的ID切换问题，SORT缺乏相应的长关联机制。
