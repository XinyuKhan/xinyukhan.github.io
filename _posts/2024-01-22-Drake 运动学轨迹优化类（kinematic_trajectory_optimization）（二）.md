## Drake 运动学轨迹优化类（kinematic_trajectory_optimization）（二）

# 1. 背景

本文继续上文的话题，继续分析`kinematic_trajectory_optimization`类的具体代码实现。

这部分的代码位于下面两个文件中：

```bash
planning\trajectory_optimization\kinematic_trajectory_optimization.h
planning\trajectory_optimization\kinematic_trajectory_optimization.cc
```

回顾上一篇文章的讨论，我们知道，该类的主要目的是解决如下问题：

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

并且有以下关系：

$$
\begin{aligned}
\overline{q}(\tau) &= \overline{q}(\frac{t}{T})=q(t)\\
\dot{q}(t) &= \frac{dq}{dt} = \frac{dq}{d\tau}\frac{d\tau}{dt} = \frac{\dot{\overline{q}}(\tau)}{T} \\
\ddot{q}(t) &= \frac{d^2q}{dt^2} = \frac{d}{dt}(\frac{dq}{dt}) = \frac{d}{d\tau}(\frac{dq}{d\tau})\frac{d\tau}{dt} = \frac{\ddot{\overline{q}}(\tau)}{T^2}
\end{aligned}
\tag{2}
$$

经过转化，我们可以得到如下形式的优化问题：

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
\tag{3}
$$

**（为了方便起见，下文用$q$表示$\overline{q}$，用$\dot{q}$表示$\dot{\overline{q}}$，用$\ddot{q}$表示$\ddot{\overline{q}}$）**

详细推导请参考上一篇文章。

# 2. 代码实现

## 2.1. 综述

`KinematicTrajectoryOptimization`类的主要作用，就是使用BSpline描述一个n维运动学轨迹，通过配置其代价和约束，以便用来描述运动学轨迹约束优化问题，然后使用`MathematicalProgram`将该问题的描述配置，为其选择一个“合适”的优化器，并将通用的问题描述，变成变成一个特化的约束优化问题的的配置，最后通过调用求解器得到求解结果，根据求解结果，得到最优的运动学轨迹。

## 2.2. 确定优化变量

在`KinematicTrajectoryOptimization`类的实现中，我们有优化时间变量$T$和控制点变量$q$，其中控制点变量$q$是一个$n \times m$维向量，其中n表示控制点的个数，m表示每个控制点的维度。如果我们需要优化一个7自由度的机械臂的运动学轨迹，那么m=7。

下面我们看代码中是如何实现的，这部分代码主要在`KinematicTrajectoryOptimization`类的构造函数中实现：

```cpp
KinematicTrajectoryOptimization::KinematicTrajectoryOptimization(
    const trajectories::BsplineTrajectory<double>& trajectory)
    : num_positions_(trajectory.rows()),
      num_control_points_(trajectory.num_control_points()) {
  // basis_ = trajectory.basis() normalized to s∈[0,1].
  const double duration = trajectory.end_time() - trajectory.start_time();
  std::vector<double> normalized_knots = trajectory.basis().knots();
  for (auto& knot : normalized_knots) {
    knot /= duration;
  }
  basis_ = BsplineBasis<double>(trajectory.basis().order(), normalized_knots);
  ...
```

根据上述代码，我们可以看到，需要使用一个`trajectories::BsplineTrajectory<double>`类型的对象来初始化一个轨迹优化问题，该对象的具体实现已经在之前的文章中做过详细的介绍，这里不再重复，我们只需要知道，该对象可以用来描述一个BSpline轨迹，其中包含了轨迹的控制点，轨迹的时间范围，轨迹的阶数等信息。使用这些参数我们用来配置我们的轨迹优化问题的变量以及它们的初值。

接下来是优化变量的创建：


```cpp
  ...
  control_points_ = prog_.NewContinuousVariables(
      num_positions_, num_control_points_, "control_point");
  duration_ = prog_.NewContinuousVariables(1, "duration")[0];
  // duration >= 0.
  prog_.AddBoundingBoxConstraint(0, std::numeric_limits<double>::infinity(),
                                 duration_);
  ...
```

代码中的`prog_`是上文提到的`MathematicalProgram`类的对象，该类是`drake`中极其重要的的一个用来描述约束优化问题的类，它可以描述一个通用的约束优化问题，然后根据问题的“特点”将其转化为一个使用特定求解器求解的约束优化问题配置，最后调用求解器求解，得到求解结果。`MathematicalProgram`类在`drake`中作为一种对求解器的一般性抽象，为该库的求解器提供了统一的接口，使得求解器的切换变得非常容易。我会在以后的内容中对它的实现细节进行单独的分析。

我们注意到，在我们现有的代码中，构造器通过`prog_`对象调用`NewContinuousVariables`方法，创建了上文提到的优化变量$T$和$q$的对象，并且通过`AddBoundingBoxConstraint`方法，为$T$添加了一个非负约束。

你也许会好奇，这个所谓的“优化变量”对象的实质到底是什么呢，要想明白这个问题，我们就必须提到`drake`实现中另一个关键概念，那就是**符号（Symbolic）**。它是一种特殊的类型对象，`drake`构建了一套自己的symblic计算框架，其中涉及到 **变量（Variable）** 、 **表达式（Expression）** 多个基本概念（类）,这些类是一套可以和Eigen配合使用的相对完备的模板系统，可以用来定义变量、完成描述约束代价、代入求值、微分求导等重要工作，在`drake`的代码实现中发挥着关键的基础性的重要作用。这些类的实现细节超出了本文的讨论范围，我会在以后的内容中对它们的实现细节进行单独的分析。我们目前只需要知道`NewContinuousVariables`方法可以返回一个包含`Variable`类型对象的Eigen向量或者矩阵，并且会在`prog_`对象中注册这些变量，这样我们就可以在后续的代码中使用这些变量了。同理，`AddBoundingBoxConstraint`方法可以为`Variable`类型对象添加一个约束，这些被添加的约束也会被注册到`prog_`对象中，这样我们就可以在后续的代码中使用这些约束了。

下面我们看构造器中的剩余部分：

```cpp
  ...
  SetInitialGuess(trajectory);

  // Create symbolic curves to enable creating linear constraints on the
  // positions and its derivatives.
  // TODO(russt): Consider computing these only the first time they are used.
  sym_r_ = std::make_unique<BsplineTrajectory<symbolic::Expression>>(
      basis_, EigenToStdVector<Expression>(control_points_.cast<Expression>()));
  sym_rdot_ =
      dynamic_pointer_cast_or_throw<BsplineTrajectory<symbolic::Expression>>(
          sym_r_->MakeDerivative());
  sym_rddot_ =
      dynamic_pointer_cast_or_throw<BsplineTrajectory<symbolic::Expression>>(
          sym_rdot_->MakeDerivative());
  sym_rdddot_ =
      dynamic_pointer_cast_or_throw<BsplineTrajectory<symbolic::Expression>>(
          sym_rddot_->MakeDerivative());
}
```
`SetInitialGuess`使用`trajectory`对象中的控制点和时间信息，为优化变量的初值赋值：

```cpp
void KinematicTrajectoryOptimization::SetInitialGuess(
    const trajectories::BsplineTrajectory<double>& trajectory) {
  prog_.SetInitialGuess(duration_, trajectory.end_time() -
                                     trajectory.start_time());
  prog_.SetInitialGuess(control_points_,
                        EigenToStdVector(trajectory.get_control_points()));
}
```

后边的代码中，我们看到，`sym_r_`、`sym_rdot_`、`sym_rddot_`、`sym_rdddot_`都是`BsplineTrajectory`模板类的`Expression`模板参数的特化，其中`sym_r_`是以`trajectory`轨迹的`basis_`作为基底，以优化符号变量对象`control_points_`作为控制点，创建的一个`BsplineTrajectory`对象，这个新的符号版本的`BsplineTrajectory<Expression>`对象，和原本的`BsplineTrajectory<double>`对象`trajectory`的区别在于它的控制点是由符号变量组成的，而不是原本的`double`类型的数值。这样我们就可以在后续的代码中，使用这些符号变量来描述约束和代价了。`sym_rdot_`、`sym_rddot_`、`sym_rdddot_`是`sym_r_`的一阶、二阶、三阶导数，它们通过`MakeDerivative`符号求导的方式得到，这个内容在之前的文章中有比较详细的介绍，这里不再赘述，总结起来就是`sym_r_`、`sym_rdot_`、`sym_rddot_`、`sym_rdddot_`都是变量`control_points_`的函数。这些在我们后边构建约束和代价的时候，会用到。


## 2.3. 确定代价

下面我们看如何确定代价cost。

### 2.3.1. 关于时间的代价

关于时间的代价很好理解，我们在在做轨迹优化的时候，一般需要时间尽可能的短，因此我们只需要使用一个 **线性代价（Linear Cost）** 来描述关于时间的代价，代码如下：

```cpp
void KinematicTrajectoryOptimization::AddDurationCost(double weight) {
  prog_.AddLinearCost(weight * duration_);
}
```

### 2.3.2. 关于长度的代价

关于轨迹长度的代价，`KinematicTrajectoryOptimization`代码中有是采用 **二阶范数（L2 Norm）** 的形式来描述，具体的方法有两种：
1. 直接使用控制点之间的差值的二阶范数的累加作为路径代价
2. 引入一个松弛变量$s$，使用$s$的**线性代价（Linear Cost）** 结合一个松弛变量和控制点之间的一个 **洛仑兹锥约束** 来描述路径代价，具体代码实现如下：

```cpp
void KinematicTrajectoryOptimization::AddPathLengthCost(
    double weight, bool use_conic_constraint) {
  MatrixXd A(num_positions_, 2 * num_positions_);
  A.leftCols(num_positions_) =
      weight * MatrixXd::Identity(num_positions_, num_positions_);
  A.rightCols(num_positions_) =
      -weight * MatrixXd::Identity(num_positions_, num_positions_);
  const VectorXd b = VectorXd::Zero(num_positions_);
  VectorXDecisionVariable vars(2 * num_positions_);
  for (int i = 1; i < num_control_points(); ++i) {
    vars.head(num_positions_) = control_points_.col(i);
    vars.tail(num_positions_) = control_points_.col(i - 1);
    if (use_conic_constraint) {
      prog_.AddL2NormCostUsingConicConstraint(A, b, vars);
    } else {
      prog_.AddL2NormCost(A, b, vars);
    }
  }
}
```

代码中首先构造了一个$n \times 2n$的矩阵$A$：

$$
A = w_{length}\begin{bmatrix}
I_n & -I_n
\end{bmatrix}
\tag{4}
$$

一个全零的$n$维向量$b$：

$$
b = \begin{bmatrix}
0_n
\end{bmatrix}
\tag{5}
$$

于是得到个二阶范数的代价项：

$$
J_{length} = \sum_{i=1}^{m-1} \vert \vert A\begin{bmatrix}
q_i \\ q_{i-1}
\end{bmatrix}-b
 \vert \vert _2 = w_{length}\sum_{i=1}^{m-1} \vert \vert (q_i-q_{i-1}) \vert \vert _2
\tag{6}
$$

如果直接使用**L2NormCost**，那就直接调用`AddL2NormCost`方法并将上述的$A$和$b$以及相关的变量向量传入即可，如果使用**L2NormCostUsingConicConstraint**，那么就需要将上述的$A$和$b$以及相关的变量向量传入`AddL2NormCostUsingConicConstraint`方法，该方法会自动将其转化为一个 **洛仑兹锥约束**，并添加到`prog_`对象中。

我们一窥`AddL2NormCostUsingConicConstraint`方法的实现：

```cpp
std::tuple<symbolic::Variable, Binding<LinearCost>,
           Binding<LorentzConeConstraint>>
MathematicalProgram::AddL2NormCostUsingConicConstraint(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  auto s = this->NewContinuousVariables<1>("slack")(0);
  auto linear_cost =
      this->AddLinearCost(Vector1d(1), 0, Vector1<symbolic::Variable>(s));
  // A_full = [1 0]
  //          [0 A]
  // b_full = [0 b]
  // A_full * [s ; vars] + b_full = [s, A*vars+b]
  Eigen::MatrixXd A_full(A.rows() + 1, A.cols() + 1);
  A_full.setZero();
  A_full(0, 0) = 1;
  A_full.bottomRightCorner(A.rows(), A.cols()) = A;
  Eigen::VectorXd b_full(b.rows() + 1);
  b_full(0) = 0;
  b_full.bottomRows(b.rows()) = b;
  auto lorentz_cone_constraint = this->AddLorentzConeConstraint(
      A_full, b_full, {Vector1<symbolic::Variable>(s), vars});
  return std::make_tuple(s, linear_cost, lorentz_cone_constraint);
}

```
上述代码引入了一个新的松弛变量$s$，将其作为一个作为一个新的优化变量，并使用它的线性代价作为一个代价项，然后将原本的二阶范数的代价项转化为一个 **洛仑兹锥约束**，并添加到`prog_`对象中。

首先构建一个新的$A_{full}$：

$$
A_{full} = \begin{bmatrix}
1 & 0 \\
0 & A
\end{bmatrix}
\tag{7}
$$

以及一个新的$b_{full}$：

$$
b_{full} = \begin{bmatrix}
0 \\
b
\end{bmatrix}
\tag{8}
$$

以及新的变量列表：

$$
\begin{aligned}
x_i &= \begin{bmatrix}
s \\ q_i \\ q_{i-1}
\end{bmatrix}
\end{aligned}
\tag{9}
$$

将上述对象作为`AddLorentzConeConstraint`函数的参数调用，就可以添加一个如下形式的 **洛仑兹锥约束**：

$$
\begin{aligned}
\vert \vert A_{full}x_i+b_{full} \vert \vert _2 \leq s
\end{aligned}
\tag{10}
$$

这样就完成了如下的问题变换：

$$
\begin{aligned}
& \underset{q_i,q_{i-1}}{\text{min}}
& & \vert \vert q_i-q_{i-1} \vert \vert _2 \\
\end{aligned}
\tag{11}
$$

变成：

$$
\begin{aligned}
& \underset{s,q_i,q_{i-1}}{\text{min}}
& & s \\
& \text{subject to}
& & \vert \vert A_{full}x_i+b_{full}\vert \vert _2 \leq s
\end{aligned}
\tag{12}
$$

以上就是`KinematicTrajectoryOptimization`代码中关于路径长度的代价的实现。不难发现这两种方法得到的配置在代价部分或者约束项部分都有非线性项，因此在后续的求解过程中，都需要使用非线性优化器来求解。如果我们希望尽可能的使用线性约束优化求解器，我们也可以把长度代价定义成一个二次型（Quadratic Cost），这样就可以使用线性约束优化求解器来求解了，即：

$$
J_{length} = w_{length}\sum_{i=1}^{m-1}(q_i-q_{i-1})^T(q_i-q_{i-1})
\tag{13}
$$

不过`KinematicTrajectoryOptimization`并没有采取这样的方式。


### 2.3.3. 关于能量的代价

有关能量的代价，`KinematicTrajectoryOptimization`代码中目前只有一个`TODO`，还没有实现。不过我们可以简单讨论一下这部分内容，出于简化的考虑，我们可以简单实用广义坐标$q$的一阶导的平方来代表能量，即 $ \vert \vert \dot{q} \vert \vert _2^2$ ，这样我们就可以使用一个 **二次型（Quadratic Cost）** 来描述能量代价，即：

$$
J_{energy} = w_{energy}\vert \vert\dot{q} \vert \vert _2^2
\tag{14}
$$

不过考虑到如果是机械臂的刚体运动的能量，广义速度各项的二次项在能量中的比重是不同的（主要受惯量参数的影响），因此实际的能量形式也许应该在$\dot{q}$的各项上乘以不同的权重系数。

## 2.4. 确定约束

### 2.4.1. 时间约束

可以通过`AddDurationConstraint`方法为时间添加上下界约束，代码如下：

```cpp
void KinematicTrajectoryOptimization::AddDurationConstraint(
    optional<double> lb, optional<double> ub) {
  prog_.AddBoundingBoxConstraint(
      lb.value_or(0), ub.value_or(std::numeric_limits<double>::infinity()),
      duration_);
}
```

上述代码比较直观，不需要过多解释。

### 2.4.2. 空间约束

#### 2.4.2.1 全局空间约束

可以通过`AddPositionBounds`方法为控制点添加全局的上下界约束，代码如下：

```cpp
void KinematicTrajectoryOptimization::AddPositionBounds(
    const Eigen::Ref<const VectorXd>& lb,
    const Eigen::Ref<const VectorXd>& ub) {
  DRAKE_DEMAND(lb.size() == num_positions());
  DRAKE_DEMAND(ub.size() == num_positions());
  // This leverages the convex hull property of the B-splines: if all of the
  // control points satisfy these convex constraints and the curve is inside
  // the convex hull of these constraints, then the curve satisfies the
  // constraints for all s (and therefore all t).
  for (int i = 0; i < num_control_points(); ++i) {
    prog_.AddBoundingBoxConstraint(lb, ub, control_points_.col(i));
  }
}
```

上述代码比较直观，不需要过多解释。

#### 2.4.2.1 局部空间约束

可以通过`AddPathPositionConstraint`方法为控制点添加局部的上下界约束，代码如下：

```cpp
void KinematicTrajectoryOptimization::AddPathPositionConstraint(
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub, double s) {
  DRAKE_DEMAND(lb.size() == num_positions());
  DRAKE_DEMAND(ub.size() == num_positions());
  DRAKE_DEMAND(0 <= s && s <= 1);
  const VectorX<symbolic::Expression> sym_r_value = sym_r_->value(s);
  prog_.AddLinearConstraint(lb <= sym_r_value && sym_r_value <= ub);
}

void KinematicTrajectoryOptimization::AddPathPositionConstraint(
    const std::shared_ptr<Constraint>& constraint, double s) {
  DRAKE_DEMAND(constraint->num_vars() == num_positions_);
  DRAKE_DEMAND(0 <= s && s <= 1);
  std::vector<double> basis_function_values;
  basis_function_values.reserve(basis_.order());
  std::vector<int> active_control_point_indices =
      basis_.ComputeActiveBasisFunctionIndices(s);
  const int num_active_control_points =
      static_cast<int>(active_control_point_indices.size());
  VectorXDecisionVariable var_vector(num_active_control_points *
                                     num_positions());
  for (int i = 0; i < num_active_control_points; ++i) {
    const int control_point_index = active_control_point_indices[i];
    basis_function_values.push_back(
        basis_.EvaluateBasisFunctionI(control_point_index, s));
    var_vector.segment(i * num_positions(), num_positions()) =
        control_points_.col(control_point_index);
  }
  prog_.AddConstraint(
      std::make_shared<PathConstraint>(constraint, basis_function_values),
      var_vector);
}
```

其中第一个函数是为轨迹的$s$处（局部）指定一个上下界约束，这里就用到了上边提到的`sym_r_`符号轨迹对象，我们知道，对于一个BSpline样条，如果给定一个$s$，那么该处的轨迹值就是构成该轨迹的部分（或全部）控制点线性组合，我们已知`sym_r_`对象中的每一个控制点都是一个符号变量，因此我们可以通过`sym_r_`对象的`value`方法，得到一个$s$处的轨迹值就是一个由符号变量构成的线性表达式（Expression）,而`lb <= sym_r_value && sym_r_value <= ub`这样的用法则是Symbolic模板把表达式变成一个公式（Formula），上述公式用来描述$s$处的轨迹值在`lb`和`ub`之间这个约束，最后`AddLinearConstraint`方法可以通过模板方法从`Formula`类型的约束中提取出`LinearConstraint`类型的约束，并将其添加到`prog_`对象中。该约束是一个线性约束。

第二个方法可以让使用者自定义个一个通用的约束（Generic Constrain），该通用约束的用于描述轨迹的$s$处的约束，因此被定义为约束变量`num_vars`和轨迹的维度`num_positions_`相同，但实际上，由于`s`处的取值根据样条阶数的不同，可能涉及区间范围附近的多个控制点，这些控制点我们称之为 **激活控制点** ，因此该约束的实际变量维数应该是`num_vars`的若干倍，如果激活控制点的个数是`a`，那么实际的变量维数就是`a*num_positions_`，第二个函数所做的就是计算$s$处的激活控制点，以及这些控制点对应的基函数的值（权重），然后利用一个包装约束类`PathConstraint`，将原有的`num_vars`个约束变量的约束，包装成一个新的拥有`a*num_positions_`个约束变量的约束，并将其添加到`prog_`对象中。

包装类`PathConstraint`的实现如下：

```cpp
class PathConstraint : public Constraint {
 public:
  PathConstraint(std::shared_ptr<Constraint> wrapped_constraint,
                 std::vector<double> basis_function_values)
      : Constraint(
            wrapped_constraint->num_outputs(),
            basis_function_values.size() * wrapped_constraint->num_vars(),
            wrapped_constraint->lower_bound(),
            wrapped_constraint->upper_bound()),
        wrapped_constraint_(wrapped_constraint),
        basis_function_values_(std::move(basis_function_values)) {}

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    AutoDiffVecXd y_t;
    Eval(InitializeAutoDiff(x), &y_t);
    *y = ExtractValue(y_t);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    AutoDiffVecXd x_sum = basis_function_values_[0] *
                          x.segment(0, wrapped_constraint_->num_vars());
    const int num_terms = basis_function_values_.size();
    for (int i = 1; i < num_terms; ++i) {
      x_sum += basis_function_values_[i] *
               x.segment(i * wrapped_constraint_->num_vars(),
                         wrapped_constraint_->num_vars());
    }
    wrapped_constraint_->Eval(x_sum, y);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::runtime_error(
        "PathConstraint does not support evaluation with Expression.");
  }

 private:
  std::shared_ptr<Constraint> wrapped_constraint_;
  std::vector<double> basis_function_values_;
};
```

其本质就是在评估（DoEvel）的时候，将所有的激活控制点根据`basis_function_values_`进行加权之后，使用`wrapped_constraint_`来评估，这样就完成了从`num_vars`个约束变量的约束，包装成一个新的拥有`a*num_positions_`个约束变量的约束的过程。

### 2.4.3. 速度约束

#### 2.4.3.1. 全局速度约束

可以通过`AddVelocityBounds`方法为控制点添加全局的上下界约束，代码如下：

```cpp
void KinematicTrajectoryOptimization::AddVelocityBounds(
    const Eigen::Ref<const VectorXd>& lb,
    const Eigen::Ref<const VectorXd>& ub) {
  DRAKE_DEMAND(lb.size() == num_positions());
  DRAKE_DEMAND(ub.size() == num_positions());

  // We have q̇(t) = drds * dsdt = ṙ(s) / duration, and duration >= 0, so we
  // use duration * lb <= ṙ(s) <= duration * ub.
  //
  // This also leverages the convex hull property of the B-splines: if all of
  // the control points satisfy these convex constraints and the curve is
  // inside the convex hull of these constraints, then the curve satisfies the
  // constraints for all t.
  for (int i = 0; i < sym_rdot_->num_control_points(); ++i) {
    prog_.AddLinearConstraint(sym_rdot_->control_points()[i] >=
                                  duration_ * lb &&
                              sym_rdot_->control_points()[i] <= duration_ * ub);
  }
}
```

这里用到了`sym_rdot_`对象，其原理和上文中提到的`sym_dot_`一样，`sym_rdot_->control_points()[i]`其本质就是一个以`control_points_`为变量的表达式，根据公式（3）中关于速度约束的定义，即$\dot{q}_{min}T \leq \dot{\overline{q}}(\tau) \leq \dot{q}_{max}T$，我们得到了代码中的用来描述约束的Formula，同样的`AddLinearConstraint`方法可以使用模板技术从`Formula`类对象中提取处相应的线性约束。

#### 2.4.3.2. 局部速度约束

添加局部约束的方法有两个，实现如下：

```cpp
void KinematicTrajectoryOptimization::AddPathVelocityConstraint(
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub, double s) {
  DRAKE_DEMAND(lb.size() == num_positions());
  DRAKE_DEMAND(ub.size() == num_positions());
  DRAKE_DEMAND(0 <= s && s <= 1);
  const VectorX<symbolic::Expression> sym_rdot_value = sym_rdot_->value(s);
  prog_.AddLinearConstraint(lb <= sym_rdot_value && sym_rdot_value <= ub);
}

void KinematicTrajectoryOptimization::AddVelocityConstraintAtNormalizedTime(
    const std::shared_ptr<solvers::Constraint>& constraint, double s) {
  DRAKE_DEMAND(constraint->num_vars() == 2 * num_positions_);
  DRAKE_DEMAND(0 <= s && s <= 1);

  VectorX<Expression> r = sym_r_->value(s);
  VectorX<Expression> rdot = sym_rdot_->value(s);
  VectorXDecisionVariable vars_pos, vars_vel;
  std::unordered_map<symbolic::Variable::Id, int> unused_map;
  std::tie(vars_pos, unused_map) = symbolic::ExtractVariablesFromExpression(r);
  std::tie(vars_vel, unused_map) =
      symbolic::ExtractVariablesFromExpression(rdot);
  Eigen::MatrixXd M_pos(num_positions(), vars_pos.size());
  Eigen::MatrixXd M_vel(num_positions(), vars_vel.size());
  symbolic::DecomposeLinearExpressions(r, vars_pos, &M_pos);
  symbolic::DecomposeLinearExpressions(rdot, vars_vel, &M_vel);
  VectorXDecisionVariable duration_and_vars(1 + vars_pos.size() +
                                            vars_vel.size());
  duration_and_vars << duration_, vars_pos, vars_vel;
  auto wrapped_constraint = std::make_shared<WrappedVelocityConstraint>(
      constraint, std::move(M_pos), std::move(M_vel));
  prog_.AddConstraint(wrapped_constraint, duration_and_vars);
}
```

第一个方法可以指定$s$处的上下界，不过需要注意的是，这里的速度其实是时间（Duration）归一化之后的速度，并不是真实的速度。其实根据上述全局速度约束的实现，我觉得完全可以在$s$处构建一个真实速度线性约束，不知道作者为什么没有这样做。

第二个方法`AddVelocityConstraintAtNormalizedTime`主要是用来处理通用约束的，其设计原理跟上文中提到的`AddPathPositionConstraint`方法是类似的，使用一个`WrappedVelocityConstraint`包装类对速度约束进行包装，根据代码，我们可以发现需要用户定义的通用约束的约束变量个数是`2*num_positions_`，应该是方便用户对样条的位置和速度同时进行约束。我们还可以发现在构建`WrappedVelocityConstraint`的约束变量的时候，考虑了时间变量`duration_`，因此这里的约束速度应该是真实速度。由于篇幅有限，这部分代码比较复杂，这里暂时不做展开。

### 2.4.4. 加速度约束

#### 2.4.4.1 全局加速度约束

可以使用`AddAccelerationBounds`方法为控制点添加全局的上下界约束，代码如下：

```cpp
void KinematicTrajectoryOptimization::AddAccelerationBounds(
    const Eigen::Ref<const VectorXd>& lb,
    const Eigen::Ref<const VectorXd>& ub) {
  DRAKE_DEMAND(lb.size() == num_positions());
  DRAKE_DEMAND(ub.size() == num_positions());

  // We have t = duration * s. So dsdt = 1/duration, d²sdt² = 0. Then q̈(t) =
  // r̈(s) * dsdt^2.

  // This again leverages the convex hull property to enforce the guarantee ∀t
  // by only constraining the values at the control points.
  Eigen::RowVectorXd M;
  VectorXDecisionVariable vars, duration_and_vars;
  std::unordered_map<symbolic::Variable::Id, int> unused_map;
  for (int i = 0; i < sym_rddot_->num_control_points(); ++i) {
    for (int j = 0; j < num_positions(); ++j) {
      std::tie(vars, unused_map) = symbolic::ExtractVariablesFromExpression(
          sym_rddot_->control_points()[i](j));
      M.resize(vars.size());
      symbolic::DecomposeLinearExpressions(
          Vector1<Expression>(sym_rddot_->control_points()[i](j)), vars, &M);
      auto con = std::make_shared<DerivativeConstraint>(M, 2, lb.segment<1>(j),
                                                        ub.segment<1>(j));
      duration_and_vars.resize(vars.size() + 1);
      duration_and_vars << duration_, vars;
      prog_.AddConstraint(con, duration_and_vars);
    }
  }
}
```

根据公式（3）中关于加速度约束的定义，即$\ddot{q}_{min}T^2 \leq \ddot{\overline{q}}(\tau) \leq \ddot{q}_{max}T^2$，对于加速的的约束在该问题中不再能被表示成一个线性约束，因此这里定义了一个`DerivativeConstraint`类，它继承于`Constraint`类，属于一个通用约束，用来描述一个变量的导数约束，其基本原理就是根据公式（3）中的定义，将加速度约束转化为一个关于控制点和时间的约束，然后使用`AddConstraint`方法将其添加到`prog_`对象中。

其中`ExtractVariablesFromExpression`方法用来从`Expression`中提取`Variable`，`DecomposeLinearExpressions`用于对`Expression`进行求导，得到的矩阵`M`就是就是各项导数。

#### 2.4.4.2 局部加速度约束

局部加速度约束只提供了一个`AddPathAccelerationConstraint`的方法用来添加$s$处的上下界约束，代码如下：

```cpp
void KinematicTrajectoryOptimization::AddPathAccelerationConstraint(
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub, double s) {
  DRAKE_DEMAND(lb.size() == num_positions());
  DRAKE_DEMAND(ub.size() == num_positions());
  DRAKE_DEMAND(0 <= s && s <= 1);
  const VectorX<symbolic::Expression> sym_rddot_value = sym_rddot_->value(s);
  prog_.AddLinearConstraint(lb <= sym_rddot_value && sym_rddot_value <= ub);
}
```

正如上文所提到的一样，这里的加速度其实是时间（Duration）归一化之后的加速度，并不是真实的加速度，这里不再重复。

### 2.4.5. 加加速度（jerk）约束

可以使用`AddJerkBounds`方法为控制点添加全局的上下界约束，代码如下：

```cpp
void KinematicTrajectoryOptimization::AddJerkBounds(
    const Eigen::Ref<const VectorXd>& lb,
    const Eigen::Ref<const VectorXd>& ub) {
  DRAKE_DEMAND(lb.size() == num_positions());
  DRAKE_DEMAND(ub.size() == num_positions());

  // Following the derivations above, we have d³qdt³(t) = d³rds³(s) * dsdt*3.

  // This again leverages the convex hull property to enforce the guarantee ∀t
  // by only constraining the values at the control points.
  Eigen::RowVectorXd M;
  VectorXDecisionVariable vars, duration_and_vars;
  std::unordered_map<symbolic::Variable::Id, int> map_var_to_index;
  for (int i = 0; i < sym_rdddot_->num_control_points(); ++i) {
    for (int j = 0; j < num_positions(); ++j) {
      std::tie(vars, map_var_to_index) =
          symbolic::ExtractVariablesFromExpression(
              sym_rdddot_->control_points()[i](j));
      M.resize(vars.size());
      symbolic::DecomposeLinearExpressions(
          Vector1<Expression>(sym_rdddot_->control_points()[i](j)), vars, &M);
      auto con = std::make_shared<DerivativeConstraint>(M, 3, lb.segment<1>(j),
                                                        ub.segment<1>(j));
      duration_and_vars.resize(vars.size() + 1);
      duration_and_vars << duration_, vars;
      prog_.AddConstraint(con, duration_and_vars);
    }
  }
}
```

这里使用了`sym_rdddot_`对象，约束的阶数变成了3，除此之外和加速度的全局约束的实现是一样的，这里不再重复。

实现中没有提供为jerk指定局部约束的方法，也许这在实际应用中并不常见。

## 2.5. 求解器的选择和配置

当上述的一切都配置好之后，我们就可以使用`Solve`方法来求解了，代码如下：

```cpp
  result = Solve(trajopt_.prog());
```

### 2.5.1. 求解器的选择

在上述的`Solve`方法中，会通过一个`ChooseBestSolver`方法，根据`prog`的配置，选择一个最佳的求解器，代码如下：

```cpp
MathematicalProgramResult Solve(
    const MathematicalProgram& prog,
    const std::optional<Eigen::VectorXd>& initial_guess,
    const std::optional<SolverOptions>& solver_options) {
  const SolverId solver_id = ChooseBestSolver(prog);
  drake::log()->debug("solvers::Solve will use {}", solver_id);
  std::unique_ptr<SolverInterface> solver = MakeSolver(solver_id);
  MathematicalProgramResult result{};
  solver->Solve(prog, initial_guess, solver_options, &result);
  return result;
}
```

主要的选择依据主要是`prog`所描述问题的可解性、计算的复杂度等多方面因素，例如：如果上述问题中的costs和constrains都是线性的，那么`ChooseBestSolver`会优先返回一个 **OSQP** 求解器。这里具体的细节暂时不做展开。

### 2.5.2. 求解器的配置

上述代码中在调用`Solve`方法的时候，具体的求解器实现会根据`prog`中所定义的costs和constrains，将其转化为特定求解器的配置参数。例如上边提到的 **OSQP** 求解器，就是将`prog`中的costs和constrains转化为一个 **二次规划（Quadratic Programming）** 问题，其中参数包括目标函数的二次项系数矩阵$Q$，一次项系数向量$c$，约束矩阵$A$，约束向量$b$，以及变量的上下界约束向量$lb$和$ub$。

**注意**：我发现构建solver的操作每一次调用`Solve`方法都会执行一次，这可能就会比直接使用原生求解器的效率低一些，不过这也许就是抽象的“必要代价”，不过也许可以有一些对于solver进行“微调”的机制，这样在求解问题的结构不出现大的变化的情况下使封装的结果尽可能的接近原生求解器的效率。

这里的细节也暂时不做展开。


## 2.6. 求解及轨迹恢复

我们回到轨迹优化问题本身，现在我们已经通过调用`Solve`方法，求解出来最优的控制点$q$以及时间$T$，现在我们需要根据这些去恢复出一条最优的轨迹，这里用到了`ReconstructTrajectory`方法，具体实现如下：

```cpp
BsplineTrajectory<double>
KinematicTrajectoryOptimization::ReconstructTrajectory(
    const MathematicalProgramResult& result) const {
  const double duration = result.GetSolution(duration_);
  std::vector<double> scaled_knots = basis_.knots();
  for (auto& knot : scaled_knots) {
    knot *= duration;
  }

  return BsplineTrajectory<double>(
      BsplineBasis<double>(basis_.order(), scaled_knots),
      EigenToStdVector<double>(result.GetSolution(control_points_)));
}
```

这里的`result`就是上述`Solve`方法的返回值，我们可以从`result`中获取到最优的控制点$q$以及时间$T$，然后根据`basis_`对象，将时间尺度恢复，最后构建出一条新的最优轨迹。


未经授权，禁止转载






