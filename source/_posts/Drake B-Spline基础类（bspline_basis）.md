---
title: Drake B-Spline基础类（bspline_basis）
date: 2023-11-25 00:00:00
---

## Drake B-Spline基础类（bspline_basis）

# 背景

最近在研究Drake算法库运动轨迹优化部分的代码（kinematic_trajectory_optimization），其中用到了B-Spline曲线，其中有一个类`BspineBasis`，该类提供一个B-Spine曲线的基底，用于建立参数和曲线空间的映射。


给定一组非递减的断点 t₀ ≤ t₁ ≤ ⋅⋅⋅ ≤ tₘ，阶数为 k 的 B-样条基是在这些断点上定义的 n + 1 个（其中 n = m - k）阶数为 k - 1 的分段多项式集合。这个集合的元素被称为 "B-样条"。向量 (t₀, t₁, ..., tₘ)' 被称为基的 "节点向量"，其元素被称为 "节点"。

在具有重数 p 的断点处（即在节点向量中出现 p 次的断点），**B-样条**保证具有 Cᵏ⁻ᵖ⁻¹ 连续性。

使用 **B-样条基** B 的**B-样条曲线**是将参数值在 [tₖ₋₁, tₙ₊₁] 范围内映射到向量空间 V 的参数化曲线。对于 t ∈ [tₖ₋₁, tₙ₊₁]，曲线的值由在 t 处评估的 B 的元素与 n + 1 个控制点 pᵢ ∈ V 的线性组合给出。

有关 B-样条及其用途的更多信息，请参阅 Patrikalakis 等人的文献[1]。

[1] https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node15.html

本文基于Drake的版本是v1.23.0。

# B-Spline基底

## 1. 基底的构造

### 1.1 通过阶数和节点直接构造

```cpp
  /** Constructs a B-spline basis with the specified `order` and `knots`.
  @pre `knots` is sorted in non-descending order.
  @throws std::exception if knots.size() < 2 * order. */
  BsplineBasis(int order, std::vector<T> knots); 
```

这种构造方法直接提供了阶数和节点，这里的模板参数T是节点的类型，通常是double。根据定义，需要满足的条件是节点是非递减的，且节点的个数大于等于2*order。

### 1.2 通过阶数、基底函数个数节点类型和区间构造

```cpp
  /** Constructs a B-spline basis with the specified `order`,
  `num_basis_functions`, `initial_parameter_value`, `final_parameter_value`,
  and an auto-generated knot vector of the specified `type`.
  @throws std::exception if num_basis_functions < order
  @pre initial_parameter_value ≤ final_parameter_value */
  BsplineBasis(int order, int num_basis_functions,
               KnotVectorType type = KnotVectorType::kClampedUniform,
               const T& initial_parameter_value = 0,
               const T& final_parameter_value = 1);
```

根据定义，如果想让k阶的**B-Spline**曲线的起始点和终止点的值等于第一个节点和最后一个节点的值，那么节点的前k个和节点的后k个必须是重复的，剩余中间的节点（m-2*k个）是均匀分布的，这样的类型被称为Clamped Uniform（kClampedUniform），根据上述构造函数，如果总的节点数是m，那么基底函数的个数是n=m-k，因此需要满足m-k=n>=k，即m>=2k，这里的num_basis_functions就是n，order就是k。区间的开始和结束分别是initial_parameter_value和final_parameter_value，默认是归一化的。该构造函数通过调用`MakeKnotVector`方法去生成节点，具体实现如下：

```cpp
template <typename T>
std::vector<T> MakeKnotVector(int order, int num_basis_functions,
                              KnotVectorType type,
                              const T& initial_parameter_value,
                              const T& final_parameter_value) {
  if (num_basis_functions < order) {
    throw std::invalid_argument(fmt::format(
        "The number of basis functions ({}) should be greater than or "
        "equal to the order ({}).",
        num_basis_functions, order));
  }
  // 区间的开始不能大于区间的结束
  DRAKE_DEMAND(initial_parameter_value <= final_parameter_value);
  // 节点数=基底函数数+阶数，即m=n+k
  const int num_knots{num_basis_functions + order};
  std::vector<T> knots(num_knots);
  // 计算节点间隔，区间被分成m-2*k+1份，每份的长度是区间长度除以m-2*k+1
  const T knot_interval = (final_parameter_value - initial_parameter_value) /
                          (num_basis_functions - order + 1.0);
  // 生成节点，如果是Clamped Uniform类型，那么前k个和后k个节点是重复的，中间的节点是均匀分布的
  for (int i = 0; i < num_knots; ++i) {
    if (i < order && type == KnotVectorType::kClampedUniform) {
      knots.at(i) = initial_parameter_value;
    } else if (i >= num_basis_functions &&
               type == KnotVectorType::kClampedUniform) {
      knots.at(i) = final_parameter_value;
    } else {
      knots.at(i) = initial_parameter_value + knot_interval * (i - (order - 1));
    }
  }
  // 返回节点
  return knots;
}
```

## 2. 根据基底和参数计算曲线上的点

`BsplineBasis`类通过`EvaluateCurve`方法计算曲线上的点，曲线上的点的具体形式通过模板参数`T_control_point`确定，因此，曲线的具体维度和形式是不固定的。该方法的实现如下：

```cpp
  /** Evaluates the B-spline curve defined by `this` and `control_points` at the
  given `parameter_value`.
  @param control_points Control points of the B-spline curve.
  @param parameter_value Parameter value at which to evaluate the B-spline
  curve defined by `this` and `control_points`.
  @pre control_points.size() == num_basis_functions()
  @pre parameter_value ≥ initial_parameter_value()
  @pre parameter_value ≤ final_parameter_value() */
  template <typename T_control_point>
  T_control_point EvaluateCurve(
      const std::vector<T_control_point>& control_points,
      const T& parameter_value) const {
    /* This function implements the de Boor algorithm. It uses the notation
    from Patrikalakis et al. [1]. Since the depth of recursion is known
    a-priori, the algorithm is flattened along the lines described in [2] to
    avoid duplicate computations.

     [1] https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node18.html
     [2] De Boor, Carl. "On calculating with B-splines." Journal of
         Approximation theory 6.1 (1972): 50-62.

    NOTE: The implementation of this method is included in the header so that
    it can be used with custom values of T_control_point. */
    DRAKE_DEMAND(static_cast<int>(control_points.size()) ==
                 num_basis_functions());
    DRAKE_DEMAND(parameter_value >= initial_parameter_value());
    DRAKE_DEMAND(parameter_value <= final_parameter_value());

    // Define short names to match notation in [1].
    const std::vector<T>& t = knots();
    const T& t_bar = parameter_value;
    const int k = order();

    /* Find the index, 𝑙, of the greatest knot that is less than or equal to
    t_bar and strictly less than final_parameter_value(). */
    const int ell = FindContainingInterval(t_bar);
    // The vector that stores the intermediate de Boor points (the pᵢʲ in [1]).
    std::vector<T_control_point> p(order());
    /* For j = 0, i goes from ell down to ell - (k - 1). Define r such that
    i = ell - r. */
    for (int r = 0; r < k; ++r) {
      const int i = ell - r;
      p.at(r) = control_points.at(i);
    }
    /* For j = 1, ..., k - 1, i goes from ell down to ell - (k - j - 1). Again,
    i = ell - r. */
    for (int j = 1; j < k; ++j) {
      for (int r = 0; r < k - j; ++r) {
        const int i = ell - r;
        // α = (t_bar - t[i]) / (t[i + k - j] - t[i]);
        const T alpha = (t_bar - t.at(i)) / (t.at(i + k - j) - t.at(i));
        p.at(r) = (1.0 - alpha) * p.at(r + 1) + alpha * p.at(r);
      }
    }
    return p.front();
  }
```

该方法的输入是一个控制点的向量列表（vector）以及一个参数值，输出是该参数值对应的曲线上的一个点，该点的类型是模板参数`T_control_point`。输入参数需要满足的条件是控制点的个数等于基底函数的个数（即m-k），参数值在区间内。该方法的实现是通过递归的方式实现的，具体的实现细节可以参考文献[1]和[2]。

下面我们逐一分析该方法的实现细节。

### 2.1 根据参数值找到节点向量中对应的区间索引

该过程通过成员函数`FindContainingInterval`实现，该函数的实现如下：

```cpp
template <typename T>
int BsplineBasis<T>::FindContainingInterval(const T& parameter_value) const {
  DRAKE_ASSERT(parameter_value >= initial_parameter_value());
  DRAKE_ASSERT(parameter_value <= final_parameter_value());
  const std::vector<T>& t = knots();
  const T& t_bar = parameter_value;
  return std::distance(
      t.begin(), std::prev(t_bar < final_parameter_value()
                               ? std::upper_bound(t.begin(), t.end(), t_bar,
                                                  less_than_with_cast<T>)
                               : std::lower_bound(t.begin(), t.end(), t_bar,
                                                  less_than_with_cast<T>)));
}
```

本质上就是一个在knots向量上的二分查找，不过在处理节点向量中最前k（阶数）个和最后k（阶数）个节点的地方有一些特殊的处理，主要针对的是上文提到的Clamped Uniform类型的曲线，主要是为了保证得到的区间索引落在[k,m-k]的范围内。

### 2.2 根据曲线的阶数k迭代计算Pᵢʲ，直到j=k-1，此时P₀ᵏ就是曲线上的点

这一步的具体原理可以参考上述的文献[2]，大致过程是这样的：

- 首先从上一步找到的区间索引处开始，向前迭代计算Pᵢ⁰，直到i=ell-(k-1)；
- 然后从上一步找到的区间索引处开始，向前迭代计算Pᵢ¹，直到i=ell-(k-2)；
- 依次类推，每次P的个数减少1，呈现出一种金字塔型的结构，直到Pᵢᵏ-¹；
- 最后P只剩下一个，即为曲线上的点。
  
## 3. 杂项方法

`BsplineBasis`类还提供了一些其他的方法，比如一些基本的get方法、参数检查工具方法比如`CheckInvariants`等，都相对比较简单，这里不再赘述。

除此以外，`BsplineBasis`还提供了一个`ComputeActiveBasisFunctionIndices`方法，可以根据参数值或者参数区间计算出当前活跃的基底函数的索引的列表向量，其中同样是用到了`FindContainingInterval`方法，这里不再重复。

至此，`BsplineBasis`类的主要内容就分析完了。

---

未经允许，禁止转载。

[1] https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node15.html
[2] https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node18.html
[3] De Boor, Carl. "On calculating with B-splines." Journal of
         Approximation theory 6.1 (1972): 50-62.
