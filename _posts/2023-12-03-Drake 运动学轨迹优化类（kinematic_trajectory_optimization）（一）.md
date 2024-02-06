## Drake 运动学轨迹优化类（kinematic_trajectory_optimization）（一）


# 背景

本文主要讨论运动学轨迹优化类（`KinematicTrajectoryOptimization`），该类用于解决运动学轨迹优化问题。这次先讨论理论，后续再讨论具体的实现。

这部分的代码位于下面两个文件中：

`planning\trajectory_optimization\kinematic_trajectory_optimization.h`

`planning\trajectory_optimization\kinematic_trajectory_optimization.cc`


# 问题定义

该运动学轨迹优化类用于解决如下问题：

$$
\begin{aligned}
& \underset{T,q(\cdot)}{\text{min}}
& & J(T,q(\cdot)) \\
& \text{subject to}
& & q(t_0) = q_0, \space q(t_f) = q_f \\
& & & \dot{q}(t_0) = \dot{q}_0, \space \dot{q}(t_f) = 
\dot{q}_f \\
& & & \ddot{q}(t_0) = \ddot{q}_0, \space \ddot{q}(t_f) = \ddot{q}_f \\
& & & \dot{q}_{min} \leq \dot{q}(t) \leq \dot{q}_{max} \\
& & & \ddot{q}_{min} \leq \ddot{q}(t) \leq \ddot{q}_{max} \\
& & & \text{obstacle avoidance} \\
& & & \text{kinematics, dynamics constrains}
\end{aligned}
\tag{1}
$$

上述问题中，$q(\cdot)$是一个关于时间的四次阶连续可导的函数，$q(t)$表示在时间$t$时刻的机器人的关节角度（广义坐标）。$T$是一个标量，表示整个轨迹的时间长度。我们的优化目标就是找到一个$T$和一个关于时间的函数$q(\cdot)$，在满足约束的情况下，使得$J(T,q(\cdot))$最小，其中$J(T,q(\cdot))$是一个关于$T$和$q(\cdot)$的标量函数，表示整个轨迹的代价。

## 1. 问题的转化

由于在实际应用中，计算机通常不太方便对一个连续函数进行优化，因此我们通过使用一个四次B样条曲线来近似表示$q(\cdot)$，即：

$$
q(t) = \sum_{i=0}^{n}q_iB_{i,p}(t)
\tag{2}
$$

由于$T$也是一个需要优化的变量，为了简化优化问题，我们可以将优化曲线的参数$t$归一化到$[0,1]$区间，即$\tau \in [0,1], \space \tau = \frac{t}{T}$，曲线和总时间就可以被当作两个彼此独立的变量来处理。如下：

$$
\overline{q}(\tau) = \overline{q}(\frac{t}{T})=q(t) \tag{3}
$$

求导可得：

$$
\begin{aligned}
\dot{q}(t) &= \frac{dq}{dt} = \frac{dq}{d\tau}\frac{d\tau}{dt} = \frac{\dot{\overline{q}}(\tau)}{T} \\
\ddot{q}(t) &= \frac{d^2q}{dt^2} = \frac{d}{dt}(\frac{dq}{dt}) = \frac{d}{d\tau}(\frac{dq}{d\tau})\frac{d\tau}{dt} = \frac{\ddot{\overline{q}}(\tau)}{T^2}
\end{aligned}
\tag{4}
$$

这样，我们就可以将上述优化问题转化为如下形式：

$$
\begin{aligned}
& \underset{T,\overline{q}_{0:n}}{\text{min}}
& & J(T,\overline{q}_{0:n}) \\
& \text{subject to}
& & \overline{q}(0) = q_0, \space \overline{q}(1) = q_f \\
& & & \dot{\overline{q}}(0) = \dot{q}_0T, \space \dot{\overline{q}}(1) =
\dot{q}_fT \\
& & & \ddot{\overline{q}}(0) = \ddot{q}_0T^2, \space \ddot{\overline{q}}(1) = \ddot{q}_fT^2 \\
& & & \dot{q}_{min}T \leq \dot{\overline{q}}(\tau) \leq \dot{q}_{max}T \\
& & & \ddot{q}_{min}T^2 \leq \ddot{\overline{q}}(\tau) \leq \ddot{q}_{max}T^2 \\
& & & {\frac{d^3q}{d\tau^3}}_{min}T^3 \leq \frac{d^3\overline{q}(\tau)}{d\tau^3} \leq {\frac{d^3q}{d\tau^3}}_{max}T^3 \\
& & & \text{obstacle avoidance} \\
& & & \text{kinematics, dynamics constrains}
\end{aligned}
\tag{5}
$$

**（为了方便起见，下文用$q$表示$\overline{q}$，用$\dot{q}$表示$\dot{\overline{q}}$，用$\ddot{q}$表示$\ddot{\overline{q}}$）**

## 2. 轨迹约束

### 2.1 样条

$$
\begin{aligned}
    q(\tau) &= \sum_{i=0}^{n}q_iB_{i,p}(\tau) \\
    &= \begin{bmatrix}
        B_{0,p}(\tau) & B_{1,p}(\tau) & \cdots & B_{n,p}(\tau)
    \end{bmatrix}\begin{bmatrix}
        q_0 \\ q_1 \\ \vdots \\ q_n
    \end{bmatrix}
\end{aligned}
\tag{6}
$$

根据上述公式，我们可以得知，如果我们需要在$\tau$时刻为$q$添加任边界约束，那么该约束会是一个关于控制点（$q_{0:n}$）的线性约束。

**注意**：由于$p$阶B样条曲线的当前值只与$p+1$个控制点有关，因此上述公式如果用于计算约束，那么约束项的维度应该是$p+1$，而不是$n+1$。

### 2.2 样条的一阶导数(velocity)

根据B样条的一阶导数公式：

$$
\begin{aligned}
    \dot{q}(\tau) &= \sum_{i=0}^{n}q_i\dot{B}_{i,p}(\tau) \\
    &= \begin{bmatrix}
        \dot{B}_{0,p}(\tau) & \dot{B}_{1,p}(\tau) & \cdots & \dot{B}_{n,p}(\tau)
    \end{bmatrix}\begin{bmatrix}
        q_0 \\ q_1 \\ \vdots \\ q_n
    \end{bmatrix}
\end{aligned}
\tag{7}
$$

由于$B_{i,p}(\tau)$是关于$\tau$的$p$次多项式，因此$\dot{B}_{i,p}(\tau)$是关于$\tau$的$p-1$次多项式。容易证明：

$$
\begin{aligned}
    \dot{B}_{i,p}(\tau) &= \frac{p}{\tau_{i+p}-\tau_i}B_{i,p-1}(\tau) - \frac{p}{\tau_{i+p+1}-\tau_{i+1}}B_{i+1,p-1}(\tau)
\end{aligned}
\tag{8}
$$

将上述公式代入到公式（7）中，可以得到：

$$
\begin{aligned}
    \dot{q}(\tau) &= \sum_{i=0}^{n-1}B_{i+1,p-1}(\tau)Q_i \\
    \text{where } Q_i &= \frac{p}{\tau_{i+p+1}-\tau_{i+1}}(q_{i+1}-q_i)
\end{aligned}
\tag{9}
$$

如果用矩阵表示为和$[q_0 \cdots q_n]^T$相乘的形式，可以得到：

$$
\begin{aligned}
    \dot{q}(\tau) &= \begin{bmatrix}
        B_{1,p-1}(\tau) & B_{2,p-1}(\tau) & \cdots & B_{n,p-1}(\tau)
    \end{bmatrix}\begin{bmatrix}
        Q_0 \\ Q_1 \\ \vdots \\ Q_{n-1}
    \end{bmatrix} \\
    &= \begin{bmatrix}
        B_{1,p-1}(\tau) & B_{2,p-1}(\tau) & \cdots & B_{n,p-1}(\tau)
    \end{bmatrix}\text{diag}(
        \frac{p}{\tau_{1+p}-\tau_1},\frac{p}{\tau_{2+p}-\tau_2},\cdots,\frac{p}{\tau_{n+p}-\tau_n})\begin{bmatrix}
        q_1-q_0 \\ q_2-q_1 \\ \vdots \\ q_n-q_{n-1}
    \end{bmatrix} \\
    &= \begin{bmatrix}
        B_{1,p-1}(\tau) & B_{2,p-1}(\tau) & \cdots & B_{n,p-1}(\tau)
    \end{bmatrix}\text{diag}(
        \frac{p}{\tau_{1+p}-\tau_1},\frac{p}{\tau_{2+p}-\tau_2},\cdots,\frac{p}{\tau_{n+p}-\tau_n})\begin{bmatrix}
        -1 & 1 & 0 & \cdots & 0 \\
        0 & -1 & 1 & \cdots & 0 \\
        \vdots & \vdots & \vdots & \ddots & \vdots \\
        0 & 0 & 0 & \cdots & 1
        \end{bmatrix}\begin{bmatrix}
        q_0 \\ q_1 \\ \vdots \\ q_n
        \end{bmatrix}
\end{aligned}
\tag{10}
$$

通过公式可知，$\dot{q}(\tau)$是一个$p-1$阶的B样条曲线，因此如果我们需要在$\tau$时刻为$\dot{q}$添加任边界约束，那么根据公式（5），该约束会是一个关于控制点（$q_{0:n}$）以及时间$T$的线性约束，可以描述为一个QP问题。

**注意**：由于$p-1$阶B样条曲线的当前值只与$p$个控制点有关，因此上述公式如果用于计算约束，那么约束项的维度应该是$p$，而不是$n+1$。

### 2.3 样条的二阶导数(acceleration)

根据B样条的二阶导数公式：

$$
\begin{aligned}
    \ddot{q}(\tau) &= \frac{d\dot{q}(\tau)}{d\tau} \\
    &= \frac{d}{d\tau}\left[\sum_{i=0}^{n-1}B_{i+1,p-1}(\tau)Q_i\right]\\
    &= \sum_{i=0}^{n-1}\dot{B}_{i+1,p-1}(\tau)Q_i\\
\end{aligned}
\tag{11}
$$

由此可以看出该问题和上述一阶导数的问题是一样的，只是将$p$替换为$p-1$，因此我们可以得到：

$$
\begin{aligned}
    \ddot{q}(\tau) &= \sum_{i=0}^{n-2}B_{i+2,p-2}(\tau)R_i \\
    \text{where } R_i &= \frac{p-1}{\tau_{i+p+2}-\tau_{i+2}}(Q_{i+1}-Q_i)\\
    &= \frac{p(p-1)}{(\tau_{i+p+2}-\tau_{i+2})(\tau_{i+p+1}-\tau_{i+1})}(q_{i+2}-2q_{i+1}+q_i)
\end{aligned}
\tag{12}
$$

因此我们可以得到一个相同的结论：$\ddot{q}(\tau)$是一个$p-2$阶的B样条曲线，因此如果我们需要在$\tau$时刻为$\ddot{q}$添加任边界约束，那么根据公式（5），该约束会是一个关于控制点（$q_{0:n}$）以及时间$T$的约束，不过由于其中包含$T^2$项，因此该约束不再是一个线性约束，无法描述为一个QP问题。

**注意**：由于$p-2$阶B样条曲线的当前值只与$p-1$个控制点有关，因此上述公式如果用于计算约束，那么约束项的维度应该是$p-1$，而不是$n+1$。

### 2.4 样条的三阶导数(jerk)

此处结论和上述一阶导数和二阶导数的结论是一样的，和二阶中关于约束的结论类似，如果我们需要在$\tau$时刻为$\frac{d^3q}{d\tau^3}$添加任边界约束，那么根据公式（5），该约束会是一个关于控制点（$q_{0:n}$）以及时间$T$的约束，不过由于其中包含$T^3$项，因此该约束不再是一个线性约束，无法描述为一个QP问题。

## 3. 代价函数

### 3.1 时间代价

我们希望轨迹的时间越短越好，因此我们可以将时间作为一个代价函数的一部分，即：

$$
J_{time}(T) = T
\tag{13}
$$

该代价是一个关于$T$的线性函数。

### 3.2 路径代价

我们希望轨迹的路径越短越好，因此我们可以将路径作为一个代价函数的一部分，该代价函数可以有多种形式，最为直观的一种形式是我们可以使用控制点之间的差值的二阶范数的累加作为路径代价，即：

$$
J_{path}(q_{0:n}) = \sum_{i=0}^{n-1}\|q_{i+1}-q_i\|_2
\tag{14}
$$

在控制点距离足够近的时候，上述代价可以近似反应轨迹的路径长度。由于上述代价中包含开平方运算，因此会导致优化问题无法被描述为一个QP问题，因此我们可以考虑使用控制点之间的差值的二阶范数的平方的累加作为路径代价，即：

$$
J_{path}(q_{0:n}) = \sum_{i=0}^{n-1}\|q_{i+1}-q_i\|_2^2
\tag{15}
$$

### 3.3 能量的代价

我们希望轨迹的能量越小越好，因此我们可以将能量作为一个代价函数的一部分，即：

$$
J_{energy}(q_{0:n}) = \sum_{i=0}^{n-1}\|\dot{q}_i\|_2^2
\tag{16}
$$

参考上述关于$\dot{q}(\tau)$的讨论，我们可以发现上述代价函数是一个关于控制点（$q_{0:n}$）的线性二次函数。因此是一个线性二次（Linear Quadratic）代价函数，可以使用QP问题来描述。

### 3.4 总代价

我们可以将上述三个代价函数按照一定的权重进行加权求和，得到总代价函数：

$$
J(T,q_{0:n}) = \alpha J_{time}(T) + \beta J_{path}(q_{0:n}) + \gamma J_{energy}(q_{0:n})
\tag{17}
$$



# 结论

综上所述，我们可以得到如下结论：

1. 如果我们选择时间、路径（二阶范数的平方）、能量作为代价函数，同时我们只对控制点的位置和速度进行约束，那么我们可以将该优化问题描述为一个QP问题。

2. 如果我们选择了其它的代价函数，或者我们对控制点的加速度或者加加速度进行约束，或者我们自定义的其它非线性约束，那么我们就无法将该优化问题描述为一个QP问题，只能用支持NLP（Nonlinear Programming）的优化器来求解。

---

未经授权，禁止转载







