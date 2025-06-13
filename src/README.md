# MPC-D-CBF: Dynamic Control Barrier Function-based Model Predictive Control to Safety-Critical Obstacle-Avoidance of Mobile Robot

arXiv: https://arxiv.org/abs/2209.08539

bilibili: https: https://www.bilibili.com/video/BV1fN4y1N7pD/?vd_source=e11d8557ce1350ea4930d15280abb7e2

Github: https://github.com/jianzhuozhuTHU/MPC-D-CBF

YouTube: https://youtu.be/U3X6vqKTxRw

关键变化说明：
添加松弛变量：auto slack_vars = opti.variable(N_ - 1);
为每个障碍物序列创建N_-1个松弛变量（对应每个时间步的约束）
确保松弛变量非负：opti.subject_to(slack_vars >= 0);
松弛变量必须非负，表示允许约束被违反的程度
修改约束条件：opti.subject_to(h_next + slack_vars(i) >= (1 - gamma_k) * h_curr);
原来的硬约束改为允许通过松弛变量违反约束
添加惩罚项：obj = obj + slack_weight * slack_penalty;
在目标函数中添加对松弛变量的惩罚，使得优化器尽量不违反约束
使用二次惩罚（slack_vars(i) * slack_vars(i)）比线性惩罚更有效
惩罚权重：double slack_weight = 1000.0;
权重需要足够大，以确保只有在必要时才违反约束
可以根据实际情况调整这个值
这种软约束方法的优点是，即使在原始问题不可行的情况下，优化器也能找到一个"最佳妥协"解决方案，而不是直接失败。


1. 修复的主要问题
无限循环防护：
添加了迭代计数器iteration_count
设置最大迭代次数MAX_ITERATIONS = 10000
当超过最大迭代次数时强制终止算法
数值稳定性检查：
检查h值是否为DBL_MAX或NaN
验证所有未覆盖元素都有效
防止数值溢出和下溢
输入数据验证：
在KM算法中验证距离矩阵的有效性
检查NaN、负值和无穷大
如果所有距离都无效，降级到历史匹配
2. 可能的根本原因
数据问题：calculate_dis函数可能返回了无效值（NaN、负数、无穷大）
Hungarian算法bug：step3和step5之间的无限循环
矩阵大小不匹配：输入矩阵可能有维度问题