---
title: 强化学习理论基础(3)算法(7)带基线的Actor-Critic算法（A2C算法）
date: 2025-08-12 00:11:00
---

# 强化学习理论基础-算法-带基线的Actor-Critic算法（A2C算法）

在之前的文章中，我们介绍了[《Actor-Critic算法》](/2025/08/12/强化学习理论基础(3)算法(5)Actor-Critic算法/)和[《带基线的REINFORCE算法》](/2025/08/12/强化学习理论基础(3)算法(6)带基线的REINFORCE算法/)。我们知道在策略梯度定理中引入基线（Baseline）可以有效降低方差。

如果在Actor-Critic算法中引入状态价值函数 $V_{\pi}(s)$ 作为基线，我们会得到什么样的算法呢？这就是我们要介绍的带基线的Actor-Critic算法，通常被称为优势Actor-Critic（Advantage Actor-Critic，简称A2C）算法。

## 1. 推导

在[《带基线的策略梯度定理》](/2025/08/12/强化学习理论基础(2)定理(4)带基线的策略梯度定理/)中，我们得到策略梯度的无偏估计为：

<div class="math">

$$
\begin{aligned}
  \boldsymbol g(s, a; \boldsymbol{\theta}) &\triangleq \frac{\partial \ln \pi(a \mid s; \boldsymbol{\theta})}{\partial \boldsymbol{\theta}} \cdot (Q_{\pi}(s, a) - V_{\pi}(s))
\end{aligned} \tag{1.1}
$$

</div>

其中 $Q_{\pi}(s, a) - V_{\pi}(s)$ 被称为**优势函数（Advantage Function）**，通常记为 $A_{\pi}(s, a)$。优势函数表示在状态 $s$ 下采取动作 $a$ 相比于平均水平 $V_{\pi}(s)$ 的优势。

如果按照Actor-Critic算法的思路，我们需要两个神经网络来分别拟合 $Q_{\pi}(s, a)$ 和 $V_{\pi}(s)$，这显然增加了模型的复杂度和训练的难度。为了简化问题，我们可以利用状态价值函数的定义。

根据贝尔曼方程，动作价值函数可以表示为：

<div class="math">

$$
\begin{aligned}
  Q_{\pi}(s_t, a_t) &= \mathbb{E}[R_t + \gamma V_{\pi}(S_{t+1}) \mid S_t = s_t, A_t = a_t]
\end{aligned} \tag{1.2}
$$

</div>

如果我们用实际观测到的奖励 $r_t$ 和下一步的部分状态价值函数打分代替期望，就可以得到 $Q_{\pi}(s_t, a_t)$ 的蒙特卡洛近似：

<div class="math">

$$
\begin{aligned}
  Q_{\pi}(s_t, a_t) &\approx r_t + \gamma V_{\pi}(s_{t+1})
\end{aligned} \tag{1.3}
$$

</div>

将公式(1.3)代入公式(1.1)中的优势函数，得到：

<div class="math">

$$
\begin{aligned}
  A_{\pi}(s_t, a_t) &\approx r_t + \gamma V_{\pi}(s_{t+1}) - V_{\pi}(s_t) = \delta_t
\end{aligned} \tag{1.4}
$$

</div>

这是一个非常漂亮的结论！**TD误差 $\delta_t$ 可以作为优势函数 $A_{\pi}(s_t, a_t)$ 的一个无偏估计。**

这意味着我们只需要一个状态价值网络 $v(s; \boldsymbol{\omega})$ 来拟合 $V_{\pi}(s)$，就可以同时计算TD目标和近似优势函数。这大大简化了网络结构。

### 1.1. 训练价值网络

价值网络 $v(s; \boldsymbol{\omega})$ 用来拟合状态价值函数 $V_{\pi}(s)$。根据TD算法，我们可以构建损失函数：

<div class="math">

$$
\begin{aligned}
   L(\boldsymbol{\omega}) &= \frac{1}{2} (v(s_t; \boldsymbol{\omega}) - \hat y_t)^2
\end{aligned} \tag{1.1.1}
$$

</div>

其中TD目标 $\hat y_t = r_t + \gamma v(s_{t+1}; \boldsymbol{\omega})$。令 $\delta_t = \hat y_t - v(s_t; \boldsymbol{\omega})$，更新参数的公式为：

<div class="math">

$$
\begin{aligned}
  \boldsymbol{\omega} &\leftarrow \boldsymbol{\omega} - \alpha \cdot \nabla_{\boldsymbol{\omega}} L(\boldsymbol{\omega}) \\
  &\leftarrow \boldsymbol{\omega} + \alpha \cdot \delta_t \cdot \nabla_{\boldsymbol{\omega}} v(s_t; \boldsymbol{\omega})
\end{aligned} \tag{1.1.2}
$$

</div>

### 1.2. 训练策略网络

由于 $\delta_t$ 近似表示了优势函数，我们可以将近似策略梯度写为：

<div class="math">

$$
\begin{aligned}
   \tilde{\boldsymbol{g}}(s_t, a_t; \boldsymbol{\theta}) &= \frac{\partial \ln \pi(a_t \mid s_t; \boldsymbol{\theta})}{\partial \boldsymbol{\theta}} \cdot \delta_t
\end{aligned} \tag{1.2.1}
$$

</div>

使用该近似梯度，可以通过梯度上升法更新策略网络的参数 $\boldsymbol{\theta}$：

<div class="math">

$$
\begin{aligned}
   \boldsymbol{\theta} &\leftarrow \boldsymbol{\theta} + \beta \cdot \delta_t \cdot \nabla_{\boldsymbol{\theta}} \ln \pi(a_t \mid s_t; \boldsymbol{\theta})
\end{aligned} \tag{1.2.2}
$$

</div>

## 2. 训练流程

我们把当前策略网络的参数记为 $\boldsymbol{\theta}_{now}$

当前价值网络的参数记为 $\boldsymbol{\omega}_{now}$

A2C算法的单步训练流程如下：

- 观测到当前状态 $s_t$ ，根据当前策略网络做随机抽样动作：$a_t \sim \pi(\cdot \mid s_t; \boldsymbol{\theta}_{now})$，并让智能体执行该动作 $a_t$ 。
- 从环境中观测到下一个状态 $s_{t+1}$ 和奖励 $r_t$ 。

- 让价值网络对 $s_t$ 和 $s_{t+1}$ 进行打分：

<div class="math">

$$
\begin{aligned}
   \hat v_t &= v(s_t; \boldsymbol{\omega}_{now}) \\
   \hat v_{t+1} &= v(s_{t+1}; \boldsymbol{\omega}_{now})
\end{aligned} \tag{2.1}
$$

</div>

- 计算TD目标和TD误差（即近似优势函数）：

<div class="math">

$$
\begin{aligned}
   \hat y_t &= r_t + \gamma \cdot \hat v_{t+1} \\
   \delta_t &= \hat y_t - \hat v_t
\end{aligned} \tag{2.2}
$$

</div>

- 更新价值网络参数：

<div class="math">

$$
\begin{aligned}
   \boldsymbol{\omega}_{new} &\leftarrow \boldsymbol{\omega}_{now} + \alpha \cdot \delta_t \cdot \nabla_{\boldsymbol{\omega}} v(s_t; \boldsymbol{\omega}_{now})
\end{aligned} \tag{2.3}
$$

</div>

- 更新策略网络参数：

<div class="math">

$$
\begin{aligned}
   \boldsymbol{\theta}_{new} &\leftarrow \boldsymbol{\theta}_{now} + \beta \cdot \delta_t \cdot \nabla_{\boldsymbol{\theta}} \ln \pi(a_t \mid s_t; \boldsymbol{\theta}_{now})
\end{aligned} \tag{2.4}
$$

</div>

未经允许，禁止转载。
