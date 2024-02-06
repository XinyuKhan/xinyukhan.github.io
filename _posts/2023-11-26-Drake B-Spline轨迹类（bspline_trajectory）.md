## Drake: B-Spline轨迹类（bspline_trajectory）


# 背景

之前我们已经详细介绍过了Drake库中的`BsplineBasis`B-Spline基础类，这里我们将介绍`BsplineTrajectory`B-Spline轨迹类，该类继承自抽象类`Trajectory`。

这部分的代码位于下面两个文件中：

`common/trajectories/bspline_trajectory.h`

`common/trajectories/bspline_trajectory.cc`

# BsplineTrajectory类

## 1. BsplineTrajectory的构造

### 1.2 无参数构造函数

```c++
  BsplineTrajectory() : BsplineTrajectory<T>({}, {}) {}
```

此处不必多说，会构造一个空的B-Spline轨迹。

### 1.2 使用B-Spline基底和控制点序列构造

这样的的构造函数分为两种：

```c++
  /** Constructs a B-spline trajectory with the given `basis` and
  `control_points`.
  @pre control_points.size() == basis.num_basis_functions() */
  BsplineTrajectory(math::BsplineBasis<T> basis,
                    std::vector<MatrixX<T>> control_points);
```

```c++
  template <typename U = T>
  BsplineTrajectory(math::BsplineBasis<double> basis,
                    std::vector<MatrixX<T>> control_points,
                    typename std::enable_if_t<!std::is_same_v<U, double>>* = {})
      : BsplineTrajectory(math::BsplineBasis<T>(basis), control_points) {}
```

第一种构造函数适用的情况是`math::BsplineBasis<T>`和`std::vector<MatrixX<T>>`的模板参数类型相同，即`T`。

第二个构造函数中的`U`是一个模板参数，它在这里的作用是用于类型检查。这个构造函数的目的是当`U`（默认为`T`）不是`double`类型时，将`math::BsplineBasis<double>`类型的`basis`转换为`math::BsplineBasis<T>`类型，然后调用另一个构造函数。

`typename std::enable_if_t<!std::is_same_v<U, double>>* = {}`这部分代码是SFINAE（Substitution Failure Is Not An Error）技术的一种应用。它使用`std::enable_if_t`和`std::is_same_v`来检查`U`是否与`double`类型不同。如果`U`与`double`类型不同，那么`std::enable_if_t<!std::is_same_v<U, double>>`就是一个有效的类型，这个构造函数就是一个有效的候选。如果`U`与`double`类型相同，那么`std::enable_if_t<!std::is_same_v<U, double>>`就不是一个有效的类型，这个构造函数就会被排除在候选列表之外。

这样写的目的是为了在编译时进行类型检查，以确保只有当`U`（默认为`T`）不是`double`类型时，才会使用这个构造函数。这是一种常见的模板元编程技术，用于在编译时进行类型检查和条件编译。

## 2. BsplineTrajectory的`value`函数

`value`函数的定义如下：

```c++
template <typename T>
MatrixX<T> BsplineTrajectory<T>::value(const T& time) const {
  using std::clamp;
  return basis().EvaluateCurve(control_points(),
                               clamp(time, start_time(), end_time()));
}
```

`value`函数的功能就是根据参数`time`的值来计算轨迹上一点的（广义）坐标，这里用到了`math::BsplineBasis<T>`类的`EvaluateCurve`函数，这部分内容已经在`BsplineBasis`类的代码中详细介绍过了，这里就不再赘述。值得注意的一点是，这里使用了`std::clamp`函数来确保`time`的值在`start_time()`和`end_time()`之间。

还有一个需要进一步说明的是`value`函数的模板参数`T`在一些场景中未必是一个如`double`这样的数值类型，它也可以是一个drake库中定义`Expression`类型，`Expression`是drake库中symbolic代码中的一个重要概念，它可以用来表示一个符号表达式，这样的话，`value`函数的返回值就是一个符号表达式，而不是一个数值。因此其实`clamp`函数也可能不是std::clamp中的实现，drake中为它提供了`expression`版本的实现。这样的设计在drake库中非常常见，它使得drake库中的代码可以同时支持数值计算和符号计算。这对于自动微分和符号计算非常有用，这部分内容我们会在后面的文章中详细介绍。

## 3. BsplineTrajectory的求导函数

这部分的内容我参考了下面这位作者的文章：
 
https://zhuanlan.zhihu.com/p/140921657

推导过程如下：

首先B-Spline曲线的定义如下：

$$
\mathbf P(u)=\sum_{i=0}^{n} N_{i, p}(u) \mathbf P_i
$$

其中 $N_{i, p}(u)$ 是B-Spline基底函数， $\mathbf P_i$ 是控制点，$p$是B-Spline基底函数的阶数，$n$是控制点的个数。

我们假设$u$是一个时间变量，那么B-Spline曲线的导数可以表示为：

$$
\mathbf P^{\prime}(u)=\sum_{i=0}^{n} N_{i, p}^{\prime}(u) \mathbf P_i
$$

其中 $N_{i, p}^{\prime}(u)$ 是 $N_{i, p}(u)$ 的导数。

我们知道B-Spline基底函数的导数可以表示为：

$$
N_{i, p}^{\prime}(u)=\frac{p}{u_{i+p}-u_{i}} N_{i, p-1}(u)-\frac{p}{u_{i+p+1}-u_{i+1}} N_{i+1, p-1}(u)
$$

其中$u_{i}$是节点向量，$u_{i+p}$是节点向量中的第$i+p$个元素。

因此B-Spline曲线的导数可以表示为：

$$
\begin{aligned}
\mathbf{P}^{\prime}(u)&=\sum_{i=0}^{n} \frac{p}{u_{i+p}-u_{i}} N_{i, p-1}(u) \mathbf{P}_{i}-\sum_{i=0}^{n} \frac{p}{u_{i+p+1}-u_{i+1}} N_{i+1, p-1}(u) \mathbf{P}_{i+1} \\
&=\sum_{i=0}^{n-1}N_{i+1,p-1}(u)\mathbf{Q}_i \\
\mathbf{Q}_i&=\frac{p}{u_{i+p+1}-u_{i+1}} (\mathbf{P}_{i+1}-\mathbf{P}_{i})
\end{aligned}
$$

代码部分的实现在`DoMakeDerivative`成员函数中：

```c++
template <typename T>
std::unique_ptr<Trajectory<T>> BsplineTrajectory<T>::DoMakeDerivative(
    int derivative_order) const {
  if (derivative_order == 0) {
    return this->Clone();
  } else if (derivative_order > basis_.degree()) {
    std::vector<T> derivative_knots;
    derivative_knots.push_back(basis_.knots().front());
    derivative_knots.push_back(basis_.knots().back());
    std::vector<MatrixX<T>> control_points(1, MatrixX<T>::Zero(rows(), cols()));
    return std::make_unique<BsplineTrajectory<T>>(
        BsplineBasis<T>(1, derivative_knots), control_points);
  } else if (derivative_order > 1) {
    return this->MakeDerivative(1)->MakeDerivative(derivative_order - 1);
  } else if (derivative_order == 1) {
    std::vector<T> derivative_knots;
    const int num_derivative_knots = basis_.knots().size() - 2;
    derivative_knots.reserve(num_derivative_knots);
    for (int i = 1; i <= num_derivative_knots; ++i) {
      derivative_knots.push_back(basis_.knots()[i]);
    }
    std::vector<MatrixX<T>> derivative_control_points;
    derivative_control_points.reserve(num_control_points() - 1);
    for (int i = 0; i < num_control_points() - 1; ++i) {
      derivative_control_points.push_back(
          basis_.degree() /
          (basis_.knots()[i + basis_.order()] - basis_.knots()[i + 1]) *
          (control_points()[i + 1] - control_points()[i]));
    }
    return std::make_unique<BsplineTrajectory<T>>(
        BsplineBasis<T>(basis_.order() - 1, derivative_knots),
        derivative_control_points);
  } else {
    throw std::invalid_argument(
        fmt::format("Invalid derivative order ({}). The derivative order must "
                    "be greater than or equal to 0.",
                    derivative_order));
  }
}
```

这是我最喜欢的一部分，该函数传递一个参数`derivative_order`，它表示要求导的阶数。这个函数的功能是返回一个新的`Trajectory`对象，它是原来的`Trajectory`对象的导数。这个函数的实现非常巧妙，它使用了递归的思想，将求导的问题转化为了求导一次的问题。这样的设计使得代码的实现非常简洁。

值得注意的是，对于一个现有的BsplineTrajectory对象，我们每对它进行一次求导，他的阶数就会减少1，当求导的阶数大于曲线的阶数的时候，我们就会得到一个阶数为1，只有一个全零的控制点的曲线，这个曲线的导数就是0。因此我们在这里对这种情况进行了特殊处理。

如果求导阶数为1，那么就按照上述公式进行求导，如果求导阶数大于1，那么就递归调用`MakeDerivative`函数，直到求导阶数为1。

根据上述实现我们可以得知，如果我们想对一个`Trajectory`对象进行多次求导，那么我们最好是依次对他求1阶导数，而不是直接求多阶导数，这样的实现效率会更高。

同样需要说明的是，这里的`T`类型也可以是一个`Expression`类型，这样得到的导数曲线同样是使用`Expression`类型作为模板参数，这对于优化问题中对导数曲线的约束和其目标函数的定义具有重要作用。

不过还有一种情形是我们只想计算曲线上某一个点的导数，而不是整条曲线的导数，这种情形下我们可以使用`DoEvalDerivative`函数，该函数的定义如下：

```c++
template <typename T>
MatrixX<T> BsplineTrajectory<T>::DoEvalDerivative(const T& time,
                                                  int derivative_order) const {
  if (derivative_order == 0) {
    return this->value(time);
  } else if (derivative_order >= basis_.order()) {
    return MatrixX<T>::Zero(rows(), cols());
  } else if (derivative_order >= 1) {
    using std::clamp;
    T clamped_time = clamp(time, start_time(), end_time());
    // For a bspline trajectory of order n, the evaluation of k th derivative
    // should take O(k^2) time by leveraging the sparsity of basis value.
    // This differs from DoMakeDerivative, which takes O(nk) time.
    std::vector<T> derivative_knots(basis_.knots().begin() + derivative_order,
                                    basis_.knots().end() - derivative_order);
    BsplineBasis<T> lower_order_basis =
        BsplineBasis<T>(basis_.order() - derivative_order, derivative_knots);
    std::vector<MatrixX<T>> coefficients(control_points());
    std::vector<int> base_indices =
        basis_.ComputeActiveBasisFunctionIndices(clamped_time);
    for (int j = 1; j <= derivative_order; ++j) {
      for (int i = base_indices.front(); i <= base_indices.back() - j; ++i) {
        coefficients.at(i) =
            (basis_.order() - j) /
            (basis_.knots()[i + basis_.order()] - basis_.knots()[i + j]) *
            (coefficients[i + 1] - coefficients[i]);
      }
    }
    std::vector<MatrixX<T>> derivative_control_points(
        num_control_points() - derivative_order,
        MatrixX<T>::Zero(rows(), cols()));
    for (int i :
         lower_order_basis.ComputeActiveBasisFunctionIndices(clamped_time)) {
      derivative_control_points.at(i) = coefficients.at(i);
    }
    return lower_order_basis.EvaluateCurve(derivative_control_points,
                                           clamped_time);
  } else {
    throw std::invalid_argument(
        fmt::format("Invalid derivative order ({}). The derivative order must "
                    "be greater than or equal to 0.",
                    derivative_order));
  }
}
```

上述函数同样是使用上述求导公式进行求导，区别在于这里只利用`BsplineBasis`类中的`ComputeActiveBasisFunctionIndices`函数来计算曲线上与求导相关的基底函数的索引，并计算这一点的导数，而不是对整条曲线进行求导。这样的实现效率会更高。

4. 插入一个节点

`BsplineTrajectory`类支持在现有曲线的基础上插入一个或者多个节点，同时保证插入节点后的曲线与原来的曲线相同。所插入的节点必须在曲线的时间区间范围之内。该功能的实现在`InsertKnots`函数中：

```c++
template <typename T>
void BsplineTrajectory<T>::InsertKnots(const std::vector<T>& additional_knots) {
  if (additional_knots.size() != 1) {
    for (const auto& time : additional_knots) {
      InsertKnots(std::vector<T>{time});
    }
  } else {
    // Implements Boehm's Algorithm for knot insertion as described in by
    // Patrikalakis et al. [1], with a typo corrected in equation 1.76.
    //
    // [1] http://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node18.html

    // Define short-hand references to match Patrikalakis et al.:
    const std::vector<T>& t = basis_.knots();
    const T& t_bar = additional_knots.front();
    const int k = basis_.order();
    DRAKE_DEMAND(start_time() <= t_bar && t_bar <= end_time());

    /* Find the index, 𝑙, of the greatest knot that is less than or equal to
    t_bar and strictly less than end_time(). */
    const int ell = basis().FindContainingInterval(t_bar);
    std::vector<T> new_knots = t;
    new_knots.insert(std::next(new_knots.begin(), ell + 1), t_bar);
    std::vector<MatrixX<T>> new_control_points{this->control_points().front()};
    for (int i = 1; i < this->num_control_points(); ++i) {
      T a{0};
      if (i < ell - k + 2) {
        a = 1;
      } else if (i <= ell) {
        // Patrikalakis et al. have t[l + k - 1] in the denominator here ([1],
        // equation 1.76). This is contradicted by other references (e.g. [2]),
        // and does not yield the desired result (that the addition of the knot
        // leaves the values of the original trajectory unchanged). We use
        // t[i + k - 1], which agrees with [2] (modulo changes in notation)
        // and produces the desired results.
        //
        // [2] Prautzsch, Hartmut, Wolfgang Boehm, and Marco Paluszny. Bézier
        // and B-spline techniques. Springer Science & Business Media, 2013.
        a = (t_bar - t[i]) / (t[i + k - 1] - t[i]);
      }
      new_control_points.push_back((1 - a) * control_points()[i - 1] +
                                   a * control_points()[i]);
    }
    // Note that since a == 0 for i > ell in the loop above, the final control
    // point from the original trajectory is never pushed back into
    // new_control_points. Do that now.
    new_control_points.push_back(this->control_points().back());
    control_points_.swap(new_control_points);
    basis_ = BsplineBasis<T>(basis_.order(), new_knots);
  }
}
```

上述代码应用了参考文献[1]中的Boehm算法，该算法的目的是在不改变曲线形状的情况下，向曲线中插入一个节点。该算法的具体实现如下：

1. 找到一个最大的节点$t_l$，使得$t_l\leq t_{bar}$，同时$t_l<t_{end}$，这里$t_{end}$是曲线的结束时间。
2. 将节点$t_{bar}$插入到节点向量$t$中，得到新的节点向量$t^{'}$。
3. 根据节点向量$t^{'}$，计算新的控制点向量$P^{'}$，这里的计算公式如下：
4. $$
\begin{aligned}
   P^{'}_i&=\left\{\begin{aligned} P_i\quad &i\leq l-k+1\\ (1-a)P_{i-1}+aP_i\quad &l-k+2\leq i\leq l\\ P_{i-1}\quad &i>l \end{aligned}\right.\\
    a&=\frac{t_{bar}-t_i}{t_{i+k-1}-t_i}
\end{aligned}
   $$
5. 将新的节点向量和控制点向量赋值给原来的节点向量和控制点向量。
6. 重新计算B-Spline基底。
7. 重复上述步骤，直到所有的节点都被插入。


这一部分的公式推导暂时没有验证，有待后续补充。

---

# 参考文献

[1] Patrikalakis, Nicholas M., Takashi Maekawa, and Jianmin Zhao. "Shape interrogation for computer aided design and manufacturing." Springer Science & Business Media, 2009.

[2] Prautzsch, Hartmut, Wolfgang Boehm, and Marco Paluszny. Bézier and B-spline techniques. Springer Science & Business Media, 2013.

[3] https://zhuanlan.zhihu.com/p/140921657


---

未经允许，禁止转载。