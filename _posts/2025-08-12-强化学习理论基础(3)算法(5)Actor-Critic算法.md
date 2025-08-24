# 强化学习理论基础-算法-Actor-Critic算法

在[《策略梯度定理》](https://xinyukhan.github.io/2025/08/12/强化学习理论基础(2)定理(3)策略梯度定理.html)一文中我们证明推导了策略梯度定理，现在把它抄到下边以供回忆


<div class="math">

$$
\begin{aligned}
  \frac{\partial J(\boldsymbol{\theta})}{\partial \boldsymbol{\theta}} &= \frac{1 - \gamma^n}{1 - \gamma} \mathbb{E}_{S \sim d(\cdot)} \left [\mathbb{E}_{A \sim \pi(\cdot \mid S; \boldsymbol{\theta})} \left[ \frac{\partial \ln \pi(A \mid S; \boldsymbol{\theta})}{\partial \boldsymbol{\theta}} \cdot Q_{\pi}(S, A) \right]\right]
\end{aligned} \tag {0.1}
$$

</div>

同时我们介绍了近似策略梯度，即使用蒙特卡洛近似表示上述期望，得到一个随机策略梯度函数$g$

<div class="math">

$$
\begin{aligned}
   \boldsymbol{g}(s, a; \boldsymbol{\theta}) &\triangleq \frac{\partial \ln \pi(a \mid s; \boldsymbol{\theta})}{\partial \boldsymbol{\theta}} \cdot Q_{\pi}(s, a)
\end{aligned} \tag{0.2}
$$

</div>


接下来我们使用上述方法来推导Actor-Critic算法。

## 1. 推导

Actor-Critic算法中有一个价值网络$q(s, a; \boldsymbol{\omega})$和一个策略网络$\pi(a \mid s; \boldsymbol{\theta})$，其中价值网络$q(s, a; \boldsymbol{\omega})$用来近似随机策略梯度中的$Q_{\pi}(s, a)$，来解决$Q_{\pi}(s, a)$无法被直接观测到的问题。策略网络$\pi(a \mid s; \boldsymbol{\theta})$用来生成动作。Actor-Critic算法可以翻译成“演员-评委”算法，策略网络$\pi(a \mid s; \boldsymbol{\theta})$充当“演员”，生成动作；而价值网络$q(s, a; \boldsymbol{\omega})$充当“评委”，评估动作的价值。

### 1.1 训练策略网络

将上述随机策略梯度函数$g(s, a; \boldsymbol{\theta})$中的$Q_{\pi}(s, a)$替换为价值网络$q(s, a; \boldsymbol{\omega})$，得到对随机策略梯度的估计$\hat g$：

<div class="math">

$$
\begin{aligned}
   \hat g(s, a; \boldsymbol{\theta}) &\triangleq \frac{\partial \ln \pi(a \mid s; \boldsymbol{\theta})}{\partial \boldsymbol{\theta}} \cdot q(s, a; \boldsymbol{\omega})
\end{aligned}  \tag{1.1.1}
$$

</div>

然后使用该估计通过梯度上升的方法来更新策略网络的参数$\boldsymbol{\theta}$：

<div class="math">

$$
\begin{aligned}
   \boldsymbol{\theta} &\leftarrow \boldsymbol{\theta} + \beta \cdot \hat g(s, a; \boldsymbol{\theta})
\end{aligned} \tag{1.1.2}
$$

</div>

### 1.2 训练价值网络

价值网络的训练使用的是SARSA算法，在之前的文章[《SARSA算法》](https://xinyukhan.github.io/2025/08/12/强化学习理论基础(3)算法(3)SARSA算法.html)中有详细介绍。

首先在当前时刻观测到状态$s_t$，然后根据当前策略选择动作$a_t$，并执行该动作，获得奖励$r_t$和下一个状态$s_{t+1}$，接着在状态$s_{t+1}$下模拟选择动作$\tilde{a}_{t+1}$，据此我们可以计算$t$时刻的价值网络输出和$t+1$时刻（模拟）的价值网络输出并计算TD目标和TD误差：

<div class="math">

$$
\begin{aligned}
   \hat q_t &= q(s_t, a_t; \boldsymbol{\omega}) \\
   \hat q_{t+1} &= q(s_{t+1}, \tilde{a}_{t+1}; \boldsymbol{\omega}) \\
   \hat y_t &\triangleq r_t + \gamma \cdot \hat q_{t+1} \\
   \delta_t &\triangleq \hat q_t - \hat y_t
\end{aligned} \tag{1.2.1}
$$

</div>

根据SARSA算法的更新公式，价值网络的参数$\boldsymbol{\omega}$通过梯度下降的方法进行更新：

<div class="math">

$$
\omega \leftarrow \omega - \alpha \cdot \delta_t \cdot \nabla_{\boldsymbol{\omega}} q(s_t, a_t; \boldsymbol{\omega}) \tag{1.2.2}
$$

</div>


## 2. 训练流程

## 3. 目标网络改进训练流程
