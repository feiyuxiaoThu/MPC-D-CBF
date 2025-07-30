# `src/local_planner.cpp` 逐行分析

本文档提供了对 `MPC-D-CBF` 项目中 `src/local_planner.cpp` 文件的详细逐行代码分析。

## 1. 头文件包含

```cpp
#include "local_planner.h"
#include <algorithm>
#include <cmath>
```
- **L1-3**: 包含必要的头文件。
  - `"local_planner.h"`: 包含了 `LocalPlanner` 类的定义、成员变量和函数声明。
  - `<algorithm>`: 提供了 C++ 标准库中的算法，例如 `std::min`，在此代码中用于确保索引不越界。
  - `<cmath>`: 提供了标准的 C 数学函数库，如 `sqrt` (平方根), `atan2` (反正切), `cos` (余弦), `sin` (正弦) 等，这些是机器人运动学和几何计算的基础。

---
## 2. 辅助函数

这些是类中被其他成员函数调用的工具函数。

### `distanceGlobal`
```cpp
// 对应Python: def distance_global(c1, c2)
double LocalPlanner::distanceGlobal(const Eigen::Vector2d& c1, const Eigen::Vector2d& c2) {
    return std::sqrt((c1(0) - c2(0)) * (c1(0) - c2(0)) + (c1(1) - c2(1)) * (c1(1) - c2(1)));
}
```
- **L6-8**:
  - **功能**: 计算两个二维点 `c1` 和 `c2` 之间的欧几里得距离。
  - **实现**: 直接使用了标准的距离公式 `sqrt((x1-x2)^2 + (y1-y2)^2)`。`Eigen::Vector2d` 是一个包含两个 double 类型元素的向量。

### `normalizeAngle`
```cpp
// 角度归一化到[-π, π]范围
double LocalPlanner::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
```
- **L11-15**:
  - **功能**: 将任意角度 `angle` (以弧度为单位) 归一化到 `[-π, π]` 的区间内。
  - **实现**: 通过循环地加或减 `2π` (一个完整的圆周)，直到角度值落入目标范围。这对于处理具有周期性的航向角至关重要，可以避免因角度从 `+π` 跳变到 `-π` 而导致的大误差。

### `angleDifference`
```cpp
// 计算两个角度之间的最小差值
double LocalPlanner::angleDifference(double angle1, double angle2) {
    double diff = angle1 - angle2;
    return normalizeAngle(diff);
}
```
- **L18-21**:
  - **功能**: 计算两个角度之间的最短角距离。例如，`350°` 和 `10°` 的最短距离是 `20°`，而不是 `340°`。
  - **实现**: 首先计算两个角度的直接差值，然后调用 `normalizeAngle` 将差值归一化，从而得到最短路径的角度差。

---
## 3. 构造函数与析构函数

### `LocalPlanner()` (构造函数)
```cpp
// 对应Python: __init__函数 (lines 17-44)
LocalPlanner::LocalPlanner() {
    N_ = 25; // MPC预测步数
    z_ = 0.0;
    replan_period_ = 0.2;
    
    goal_state_ = Eigen::MatrixXd::Zero(N_, 3);
    
    curr_state_ = Eigen::Vector3d::Zero();
    mpc_success_ = false;
    curr_state_received_ = true;
    global_path_received_ = true;

    currPoseCallback();
    obsCallback();
    globalPathCallback();

    last_input_ = Eigen::MatrixXd::Zero(N_, 2);
    last_state_ = Eigen::MatrixXd::Zero(N_ + 1, 3);
}
```
- **L24-51**:
  - **L27-29**: 初始化 MPC 的核心参数。`N_` 是预测时域长度，`z_` 是固定的高度，`replan_period_` 是规划周期。
  - **L31**: 初始化 `goal_state_` 矩阵，用于存储从全局路径中选取的、供 MPC 跟踪的局部目标点。大小为 `N x 3`，对应 `N` 个点的 `[x, y, theta]` 状态。
  - **L33-36**: 初始化内部状态标志。`curr_state_` 存储机器人当前位姿。`mpc_success_` 标记上一次 MPC 求解是否成功。`curr_state_received_` 和 `global_path_received_` 在此被硬编码为 `true`，以允许在没有外部 ROS 消息源的情况下运行测试。
  - **L38-40**: **关键**: 在这个独立的测试版本中，直接调用回调函数来加载**模拟数据**。在与 ROS 集成的版本中，这些函数通常由消息订阅器触发。
    - `currPoseCallback()`: 设置机器人的初始位姿。
    - `obsCallback()`: 创建模拟的静态和动态障碍物数据。
    - `globalPathCallback()`: 创建一条模拟的全局参考路径。
  - **L42-43**: 初始化 `last_input_` 和 `last_state_` 矩阵。它们用于缓存上一次成功求解的控制序列和状态轨迹，以便在当前求解失败时作为备用方案（回退策略）。

### `~LocalPlanner()` (析构函数)
```cpp
LocalPlanner::~LocalPlanner() {}
```
- **L53**: 析构函数。此处为空，因为 `LocalPlanner` 类管理的资源（主要是 Eigen 矩阵和 CasADi 对象）都能自动释放内存，无需手动处理。

---
## 4. 主规划逻辑

### `replanCallback`
```cpp
void LocalPlanner::replanCallback() {
    if (!chooseGoalState()) return;
    
    // ... 为 goal_state_ 计算并填充角度信息 ...
    for (int i = 0; i < N_ - 1; ++i) {
        // ...
    }
    
    // ... 归一化所有角度 ...
    
    auto [states_sol, input_sol] = mpcEllip();
    
    // ...
}
```
- **L56-108**: 这是规划器的核心入口函数，每次规划循环都会调用它。
  - **L62**: 调用 `chooseGoalState()` 从全局路径中选取 `N` 个点作为 MPC 的参考目标。如果失败（例如没有全局路径），则直接返回，不进行本次规划。
  - **L66-93**: 为 `goal_state_` 中的每个点计算参考朝向角 `theta`。参考轨迹点通常只包含 `(x, y)` 位置。此循环通过计算相邻两个参考点的位置差，使用 `atan2(dy, dx)` 来确定路径的切线方向，并将其作为机器人的参考朝向。此部分还包含了处理角度跳变的逻辑，以确保参考角度序列的连续性。
  - **L96-98**: 再次调用 `normalizeAngle` 归一化所有计算出的参考角度。
  - **L101**: 调用 `mpcEllip()` 函数，这是执行 MPC 优化的核心。它返回求解得到的状态轨迹 `states_sol` 和控制序列 `input_sol`。C++17 的结构化绑定 `auto [...]` 用于方便地接收返回的 `std::pair`。

---
## 5. 模拟数据回调函数

在测试版本中，这些函数用于生成模拟数据，替代了 ROS 的话题订阅。

### `currPoseCallback`
- **L111-119**: 设置机器人的初始状态为 `(x=0, y=0, theta=0)`。`std::lock_guard` 用于保证对 `curr_state_` 访问的线程安全。

### `obsCallback`
- **L122-141**: 创建用于测试的障碍物数据。
  - `obstacles_.clear()`: 每次调用时清空旧数据。
  - **静态障碍物**: 创建一个位置固定的椭圆障碍物，并将其在所有 `N` 个预测时间步的位置（保持不变）添加到 `obstacles_` 列表中。
  - **动态障碍物**: 创建一个椭圆障碍物，并模拟它以恒定速度 `vx = 5.0` 直线运动，将其未来 `N` 个时间步的预测位置依次添加到 `obstacles_` 列表中。

### `globalPathCallback`
- **L144-156**: 创建一条模拟的全局参考路径。这里生成的是一条沿着 x 轴正方向延伸的直线路径。

---
## 6. MPC 核心算法与辅助函数

### `chooseGoalState`
- **L159-187**:
  - **功能**: 从全局路径中为 MPC 选取一个局部的、前瞻性的参考轨迹 `goal_state_`。
  - **实现**:
    1.  在 `global_path_` 中找到距离机器人当前位置 `curr_state_` 最近的路径点，记下其索引 `num`。
    2.  从该最近点开始，沿着 `global_path_` 连续选取 `N` 个点，填充到 `goal_state_` 矩阵中。

### `systemModel`
- **L190-197**:
  - **功能**: 以 CasADi 的符号表达式形式，定义了机器人的运动学模型（独轮车模型）。
  - **输入**: `x` 是状态 `[px, py, θ]`，`u` 是控制 `[v, ω]`。
  - **输出**: 状态的变化率 `[ẋ, ẏ, θ̇]`。
  - **关键**: 使用 `casadi::MX` 类型是 CasADi 库的核心。它允许构建一个符号计算图，CasADi 可以基于这个图自动计算梯度（导数），这是所有现代非线性优化求解器的基础。

### `ellipseConstraint`
- **L200-221**:
  - **功能**: 计算机器人位置 `pos` 到一个椭圆障碍物 `ob` 的安全距离。这是**控制障碍函数 (CBF)** 中的核心函数 `h(x)`。
  - **实现**: `safe_dist` 是机器人自身的安全半径。函数内部实现了点到旋转椭圆边界距离的数学公式。返回值 `dist > 0` 表示安全，`dist < 0` 表示碰撞。

### `quadratic`
- **L224-235**:
  - **功能**: 计算二次型表达式 `x^T * A * x`。
  - **实现**: 将 Eigen 格式的矩阵 `A` 转换为 CasADi 格式，然后使用 CasADi 的符号矩阵乘法 `mtimes` 进行计算。

### `exceedOb`
- **L238-282**:
  - **功能**: 一个启发式函数，用于判断一个障碍物是否值得在 MPC 中为其添加约束。它大致判断障碍物是否在机器人的前进方向上。
  - **目的**: 如果障碍物在目标点后方很远，可以认为它在短期内不构成威胁，从而在 MPC 中忽略此障碍物的 CBF 约束，以减少优化问题的复杂度和计算量。

### `mpcEllip`
- **L285-505**: 这是整个项目的技术核心，实现了完整的非线性 MPC 优化问题的构建和求解。
  - **L299**: `try-catch` 块用于捕获求解过程中可能发生的异常（例如，当约束条件冲突导致问题无解时）。
  - **L302-307 (步骤1: 定义优化问题)**:
    - `casadi::Opti opti`: 创建一个 CasADi 的高级优化接口实例。
    - `opti.parameter(...)`: 定义优化问题的**参数**（在一次求解中保持不变的量），这里是初始状态 `opt_x0`。
    - `opti.variable(...)`: 定义优化问题的**决策变量**（求解器需要寻找最优值的量），这里是未来的状态序列 `opt_states` 和控制序列 `opt_controls`。
  - **L313-393 (步骤2: 设置约束)**:
    - **初始状态约束**: 将预测的第一个状态与机器人实际当前状态绑定。
    - **控制输入约束**: 限制线速度和角速度在物理允许的范围内。
    - **系统动力学约束**: 使用 `systemModel` 确保状态演化符合机器人模型，将 `x_k`, `u_k`, `x_{k+1}` 关联起来。
    - **CBF 安全约束**:
      - `if (!exceedOb(...))`: 如果判断障碍物“挡路”，则为其添加 CBF 约束。
      - `auto slack_vars = opti.variable(...)`: 为该障碍物创建一组**松弛变量**。
      - `opti.subject_to(h_next + slack_vars(i) >= (1 - gamma_k) * h_curr)`: 这是核心的**软约束**形式的 D-CBF 约束。它允许在必要时轻微违反原始的硬约束 `h_next >= ...`，但这种违反会受到目标函数的惩罚。这极大地提高了求解的成功率。
  - **L395-453 (步骤3: 构建目标函数)**:
    - `casadi::MX obj = 0`: 初始化一个符号表达式作为目标函数。
    - **成本累加**: 循环遍历预测时域，累加每个时间步的**路径跟踪成本**（状态误差的二次型）和**控制成本**（控制输入大小的二次型）。
    - **终端成本**: 为预测时域的最后一个状态施加一个权重更高的惩罚，以增强 MPC 的稳定性。
    - **松弛变量惩罚**: 将所有松弛变量的平方和乘以一个大权重 `slack_weight`，并加入到目标函数中。这会驱使求解器尽可能地让松弛变量为零（即满足原始的硬约束）。
    - `opti.minimize(obj)`: 声明优化目标是最小化 `obj`。
  - **L456-464 (步骤4: 求解)**:
    - `opti.solver("ipopt", ...)`: 选择 `IPOPT` (Interior Point OPTimizer) 作为非线性规划求解器，并进行相关配置（如最大迭代次数、打印级别等）。
    - `opti.set_value(opt_x0, ...)`: 为优化问题中的参数 `opt_x0` 传入当前状态的具体数值。
    - `auto sol = opti.solve()`: **执行求解**。这是最消耗计算资源的一步。
  - **L467-486 (步骤5: 提取结果)**:
    - `sol.value(...)`: 从返回的解 `sol` 中，提取出作为决策变量的 `opt_controls` 和 `opt_states` 的最优数值解。
    - 将 CasADi 格式的结果转换为更易于在 C++ 中操作的 Eigen 矩阵格式。
    - `last_input_ = u_res`, `last_state_ = state_res`: 缓存成功的解，以备后用。
  - **L489-504 (步骤6: 异常处理)**:
    - 如果 `try` 块中的代码（主要是 `opti.solve()`）抛出异常，则执行此处的**回退策略**。
    - **时间平移 (Time-shifting)**: 如果上一次求解成功，就将上一次结果（除去第一个时间步）向前“平移”一步作为本次的输出，这可以在求解失败时提供一定的控制连续性，避免机器人完全停下。

---
## 7. 主函数 (测试入口)

```cpp
int main() {
    LocalPlanner planner;
    planner.replanCallback();
    return 0;
}
```
- **L508-513**:
  - **功能**: 作为整个 C++ 程序的入口，用于独立测试 `LocalPlanner` 类的功能。
  - **实现**:
    - `LocalPlanner planner`: 创建一个 `LocalPlanner` 对象实例。其构造函数会自动初始化所有模拟数据（初始位姿、障碍物、全局路径）。
    - `planner.replanCallback()`: 调用一次重规划流程，执行一次完整的 MPC 计算，并将结果打印到控制台。
