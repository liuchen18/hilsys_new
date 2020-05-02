# Introduction

# 当前任务
- [x] 创建一个类，这个类要有一个`subscriber`来收取`/tf`这个topic上面的信息，然后存入私有变量，之后它有一个成员函数定时从私有变量中解算杆的长度，那么，在这个类中就先需要完成解算杆长度的任务。
- [ ] 装好了solidworks之后看一下stewart机构平台中心到球铰的角度
- [x] 需要写一个auto tester来检测计算的正确性，应该在这里利用listener来读`odom`到`down_link`的变换和`odom`到`wx_link`的变换，然后输入`parallel_pose_desired`和`wx_pose`
- [x] 更改`diff_derivation.m`中的`T_up_to_wx`
- [ ] 编写`SerialIKSolver`，功能需要实现，在给定`up_link`的情况下，求解`car_to_barX, barX_to_barY, barY_to_barZ`的平动距离，并求解`barZ_to_littleX, littleX_to_littleY, littleY_to_littleZ`的转动角度

# 思路

1. 首先，我先把`platform_up.STL`和`platform_down.STL`都加入到小车的模型中，然后在二者的球铰的中心处都加上一个很小的方块，或者说就是加上一个`frame`，用来表示这个点的坐标+旋转
2. 其中`up_link`是`littleY_link`的`child`，通过`fixed joint`连接，也就是说`up_link`跟`litleZ_link`也是固连的,这样`wx_link`跟`up_link`之间存在着绕着`up_link`Z轴的旋转，给逆解增加了难度
3. 而`down_link`是`car_link`的`child`，通过`fixed joint`连接。
4. 这样，在给定`wx_link`的位姿时，`up_link`会增加一个绕`litleZ_link`的旋转，这个转轴的旋转角度在优化的时候也需要有所考虑
5. 接下来需要进行优化，优化的函数是并联机构长度的变化值，优化的自变量可以是卫星末端到车子的x、y轴距离和车子本身的转动角度。之所以这么选，是因为优化的目标即在于杆的长度不超限，所以就一并联机构长度的变化值来作为目标函数。
6. 我现在在想优化的函数应该包含3项：
   1. 杆的长度
   2. 车子相对于上一个位置的位移与转角
   3. 以卫星为基准，`up_link`相对于上一个位置的转角
7. 而优化的自变量则是
   1. 车子相对于上一个位置的位移与转角
   2. 以卫星为基准，`up_link`相对于上一个位置的转角
8. 假设`down_link`位于同一面上的两个相邻球铰之间的角度是`beta_down`，`up_link`位于同一面上的两个相邻球铰之间的角度是`beta_down`。`down_link`中球铰相对于底面的位移是`down_height`，`up_link`中球铰相对于底面的位移是`up_height`
9. 假设车子相对于上一个位置移动的位移分别是`delta_x`，`delta_y`和`delta_theta`。
10. `up_link`相对于上一个位置的转角是`delta_alpha`
11. 由于`down_link`跟`car_link`是固连的，所以`down_link`和`up_link`之间的运动自由度只有沿着$x,y,z$轴的平动和绕$x,y$轴的转动（需要注意的是，这里的绕x,y轴的转动，都是相对于**当前坐标系**的转动因此需要**右乘**）

# 推导过程
以下是推导计算`down_1`到`up_1`之间变换的过程。计算杆1的长度只需要获得`down_1`到`up_1`的平动位移，然后利用$l=\sqrt{x^2+y^2+z^2}$即可计算杆的长度了
1. 从`down_1`变换到`up_1`
$$
T_{down}^{down\_1}T_{down\_1}^{up\_1}=T_{down}^{up}T_{up}^{up\_1}\\
\Rightarrow T_{down\_1}^{up\_1}=(T_{down}^{down\_1})^{-1}T_{down}^{up}T_{up}^{up\_1}\tag{1}
$$

2. 从`origin`变换到`wx`
$$
T_o^{wx}=T_o^{down}T_{down}^{up}T_{up}^{wx}\\
\Rightarrow T_{down}^{up}=(T_o^{down})^{-1}T_o^{wx}(T_{up}^{wx})^{-1}\tag{2}
$$

3. 将式$(2)$带入式$(1)$中可以得到
$$
T_{down\_1}^{up\_1}=(T_{down}^{down\_1})^{-1}(T_o^{down})^{-1}T_o^{wx}(T_{up}^{wx})^{-1}T_{up}^{up\_1}\tag{3}
$$

计算如下
1. 在式$(3)$中，$T_{down}^{down\_1}$和$T_{up}^{up\_1}$是固定的
2. $T_{down}^{down\_1}$的产生方式是先绕$z$轴进行旋转一个角度，然后沿$x$轴行进一段距离，沿$z$轴行进一段距离
3. $T_o^{down}$可以通过`ParallelPose`中的`x,y,theta`来确定。变换方式如下

```Python
T_o_to_down=Translation('x', x)*Translation('y', y)*Translation('z', CAR_HEIGHT)*Rotation('z', theta)
```
4. $T_o^{wx}$在一次计算中也是给定的，不会发生改变
5. $T_{up}^{wx}$可以通过`ParallelPose`中的`alpha`来确定，变换方式如下

```Python
T_up_to_wx = Rotation('z', alpha)
```
6. 在获得了$T_{down}^{up}$之后，我们要计算三关节的平动和三关节的转动。$T_{down}^{up}$是通过如下变换复合得到的
```Python
T_down_to_up=Translation('x', trans_x)*Translation('y', trans_y)*Translation('z', trans_z)\
* Rotation('x', rot_x) * Rotation('y', rot_y) * Rotation('z', rot_z)
```
其中的平动复合矩阵比较好求，直接读数就可以。

而转动复合矩阵比较难求，我用`sym_cal.m`脚本计算了一下，公式如下
$$
\left[
\begin{matrix}
   \cos(\beta)*\cos(\gamma), & -\cos(\beta)*\sin(\gamma), & \sin(\beta), & 0 \\
   \cos(\alpha)*\sin(\gamma) + \cos(\gamma)*\sin(\alpha)*\sin(\beta), & \cos(\alpha)*\cos(\gamma) - \sin(\alpha)*\sin(\beta)*\sin(\gamma), & -\cos(\beta)*\sin(\alpha), & 0\\
   \sin(\alpha)*\sin(\gamma) - \cos(\alpha)*\cos(\gamma)*\sin(\beta), & \cos(\gamma)*\sin(\alpha) + \cos(\alpha)*\sin(\beta)*\sin(\gamma), & \cos(\alpha)*\cos(\beta), & 0\\
   0, & 0, & 0, & 1
\end{matrix} 
\right]
$$

7. 在`SimpleOptimizer`中对于`alpha`的优化想法
   1. 在给定`wx_pose`的情况下，对于一个初始`alpha=0`，计算`up_link`的位姿


# TIPS
1. In RVIZ, `rviz/DisplayTypes/Axes` has three axises
   1. red - x 
   2. green - y 
   3. blue -z
2. 齐次变换矩阵乘法
   1. 运动是相对于**参考坐标系**而言的，**左乘**
   2. 运动是相对于**当前坐标系**(运动坐标系)而言的，**右乘**
3. 当在Gazebo和Rviz之间进行切换的时候，请不要忘记修改IKSolver中的0.18或0.36

# 模型修改意见
1. 把`up_link`移动到`littleZ_link`的child上
2. 在`littleZ_link`上放置一根竖直的link，然后在这个link上加上一根斜着的link，绕y轴旋转45度
3. 在斜着的link上放置`wx_link`,中间加绕着$z$轴转的关节