# 冗余机器人中的 Null-Space 控制

## 简介

在冗余机器人系统中，**Null-Space 控制**是一种利用多余自由度（DOF）实现次级目标的方法，同时确保完成主任务。如果机器人的自由度超过完成主任务所需的自由度（例如，执行 6-DOF 笛卡尔任务的 7-DOF 机械臂），则被认为是冗余的。

Null-Space 控制将控制空间分为两个部分：

1. **任务空间控制**：确保实现主任务（例如，末端执行器的轨迹跟踪）。
2. **Null-Space 控制**：利用多余的自由度完成次级目标（例如，关节姿态保持、奇异性规避、能量优化）。

---

## Null-Space 投影

### 关键公式

总控制力矩（\\( \\tau \\)）可以分为两部分：

\\[
\\tau = J^T \\lambda + (I - J^T J^{+}) \\tau_{\\text{null-space}}
\\]

- \\( J^T \\lambda \\)：用于主任务的力矩。
- \\( (I - J^T J^{+}) \\)：Null-Space 投影矩阵，确保次级任务不会干扰主任务。
- \\( \\tau_{\\text{null-space}} \\)：用于次级任务的力矩。
- \\( J \\)：雅可比矩阵。
- \\( J^{+} \\)：雅可比矩阵的伪逆。

### Null-Space 投影矩阵

- **\\( J^T J^{+} \\)**：投影到任务空间的分量。
- **\\( I - J^T J^{+} \\)**：投影到任务空间的正交补（即 Null-Space）。

---

## Null-Space 控制的主要用途

### 1. 关节姿态保持

**目标**：
- 保持机器人的关节在一个舒适或安全的位置，避免关节过度弯曲或达到关节极限。

**实现方法**：
- 设置目标关节位置 \\( q_d \\)。
- 在 Null-Space 中定义弹簧-阻尼系统，引导关节向目标位置运动：
  \\[
  \\tau_{\\text{null-space}} = K \\cdot (q_d - q) - D \\cdot \\dot{q}
  \\]
  - \\( K \\)：刚度。
  - \\( D \\)：阻尼。

**应用场景**：
- 在高重复任务中保持关节姿态稳定，例如制造业中的装配任务。

---

### 2. 冗余度优化

**目标**：
- 充分利用冗余自由度，优化机器人的运动性能（如最小化能耗、避免碰撞）。

**实现方法**：
- **能量最小化**：通过最小化关节力矩 \\( \\tau \\) 或关节速度 \\( \\dot{q} \\) 的二范数：
  \\[
  \\tau_{\\text{null-space}} = \\text{argmin} \\| \\tau \\|
  \\]
- **关节运动平滑性**：通过最小化关节加速度 \\( \\ddot{q} \\) 或关节速度变化。

**应用场景**：
- 服务机器人需要高效执行任务，减少电机能耗。
- 避免长时间使用某些关节以延长设备寿命。

---

### 3. 避免奇异性

**目标**：
- 调整关节姿态，避免机器人接近奇异位形，确保雅可比矩阵的可逆性。

**实现方法**：
- 定义奇异性规避函数，例如：
  \\[
  S(q) = \\text{det}(J \\cdot J^T)
  \\]
  - \\( S(q) \\) 表示奇异性指标（值越接近 0，越接近奇异）。
- 在 Null-Space 中引入控制项以远离奇异位形：
  \\[
  \\tau_{\\text{null-space}} = \\nabla_q S(q)
  \\]

**应用场景**：
- 机器人在接近其工作空间边界或奇异点时。

---

### 4. 避免碰撞

**目标**：
- 调整关节位置以避免机器人与环境发生碰撞。

**实现方法**：
- 使用距离场检测，计算机器人关节与障碍物的最近距离 \\( d \\)。
- 在 Null-Space 中定义力矩以远离障碍物：
  \\[
  \\tau_{\\text{null-space}} = -K_{\\text{collision}} \\cdot \\nabla_q d
  \\]
  - \\( K_{\\text{collision}} \\)：碰撞规避刚度系数。

**应用场景**：
- 服务机器人在动态环境中运行（如工厂车间或家庭场景）。

---

## 实现 Null-Space 控制的算法

### 控制框架

1. **获取机器人状态**：
   - 当前关节位置 \\( q \\)、速度 \\( \\dot{q} \\)、雅可比矩阵 \\( J \\) 等。
2. **计算主任务控制**：
   - 使用任务空间控制器（如阻抗控制或轨迹跟踪）。
3. **计算 Null-Space 力矩**：
   - 通过次任务（如姿态保持或奇异性规避）计算力矩。
   - 使用正交投影矩阵将次任务的控制力矩限定在 Null-Space 中。
4. **叠加控制力矩**：
   - 将主任务和次任务的控制力矩合并。

### 代码示例

```cpp
Eigen::VectorXd tau(7);               // 总控制力矩
Eigen::VectorXd tau_task(7);          // 主任务力矩
Eigen::VectorXd tau_nullspace(7);     // 次任务力矩

// 主任务控制
tau_task = jacobian.transpose() * cartesian_force;

// 次任务控制
Eigen::VectorXd q_d_nullspace = ...;  // 目标关节配置
Eigen::VectorXd q = ...;              // 当前关节位置
Eigen::VectorXd dq = ...;             // 当前关节速度
double nullspace_stiffness = 50.0;

Eigen::MatrixXd jacobian_transpose_pinv = jacobian.transpose() * (jacobian * jacobian.transpose()).inverse();

tau_nullspace = (Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv) *
                (nullspace_stiffness * (q_d_nullspace - q) -
                2.0 * sqrt(nullspace_stiffness) * dq);

// 合并控制力矩
tau = tau_task + tau_nullspace;
