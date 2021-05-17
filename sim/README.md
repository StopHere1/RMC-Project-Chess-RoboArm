# Simulation
## About the RTB codes
注释掉不需要的代码块，一张图一张图的跑，不然时间有点久；

## Trajectory Planning
轨迹规划提供了两个比较简单的方案，第一个方案是通过直线运动；
<div align = "center">
<img src = whole_traj_straightline.png width = 60%/>
</div>

第二个方案是通过弧线运动，第二个方案可能在直接写代码上会简单一些（将运动学求解简化为双连杆问题）；但是如果可以调库去算的话，第一个思路会简洁一些（直接传入两个端点，线性求解）；

<div align = "center">
<img src = whole_traj_curve.png width = 60%/>
</div>

这里我觉得直接用第二个方案好一些，下面尝试进行运动学和动力学分析

## Analysis

方案二思路：
* 给定pick_position，即目标棋子坐标（带高度height）

```matlab
height = 1; % 每个棋子有固定的高度，作为求解的传入参数
pick_chess_location = [30 20 height];
place_chess_location = [15 -10 height];

pick_angle = atan2(pick_chess_location(2), pick_chess_location(1));
place_angle = atan2(place_chess_location(2), place_chess_location(1));
```

* 到达棋子位置，执行pick，然后抬起手臂

```matlab

pick = reshape([pick_chess_location(1) * ones(1, 100) pick_chess_location(2) * ones(1, 100) linspace(height, height + 20, 100)], 100, 3);

if (pick_angle > 0 && place_angle < 0) % exclude the wrong solution(path too long)
    theta = flip(linspace(place_angle, pick_angle, 100));
else
    theta = linspace(pick_angle, place_angle, 100);
end
```

* 转动base，到达place_angle角度
* 平动致棋子上方

```matlab
center = [0 0 20]; % 圆心位置
radius = sqrt(pick_chess_location(1)^2 + pick_chess_location(2)^2);

degs = zeros(100, 3); % 求解转动的角度向量
degs(:, 1) = cos(theta)';
degs(:, 2) = sin(theta)';
degs(:, 3) = zeros(size(theta))';

% 两端运动，第一段转动，第二段平动致棋子上方
move_1 = repmat(center, 100, 1) + radius * degs;
move_2 = reshape([linspace(cos(place_angle) * radius, place_chess_location(1), 100) linspace(sin(place_angle) * radius, place_chess_location(2), 100) 20 * ones(1, 100)], 100, 3);
move = [move_1; move_2];
```

* 执行place操作

```matlab
place = reshape([place_chess_location(1) * ones(1, 100) place_chess_location(2) * ones(1, 100) linspace(height + 20, height, 100)], 100, 3);
```

* 最终共轨迹为

```matlab
traj = [pick; move; place];
```

在上述运动学过程中，可以将问题简化为一个带末端旋转的双连杆机械臂问题，和一个base的旋转问题。后者只需要获取棋子和放置位置的坐标即可计算角度

## Two link Problem
要解决的有两种运动，一种是向上平动的pick和向下平动的place;另一种是在move操作的第二阶段的stretch或withdraw的操作

<div align = "center">
<img src = repo/two_link.svg width = 60%/>
</div>

### 正运动学方程
$$ x_{end}=l_1cos(\theta_1)+l_2cos(\theta_1+\theta_2) $$
$$ y_{end}=l_1sin(\theta_1)+l_2sin(\theta_1+\theta_2) $$

### 逆运动学求解

<div align = "center">
<img src = repo/pic2.svg width = 60%/>
</div>

$$ cos(\theta_2)=\frac{x_{end}^2+y_{end}^2-l_1^2+l_2^2}{2l_1l_2} $$
\
NOTED: we need an upper elbow solution, therefore:
$$ \theta_2=atan2(-\sqrt{1-cos^2(\theta_2)},cos(\theta_2)) $$
$$ \theta_1= atan2(y_{end},x_{end})-atan2(l_2sin(\theta_2),l_1+l_2cos(\theta_2)) $$
\
For the electromagnet in the end:
$$ \theta_3=\pi-(2\pi-\theta_1-\pi+\theta_2-\pi/2)=\theta_1-\theta_2+\pi/2 $$


### 速度雅可比矩阵
还没求

### 正向动力学方程
缺实际物理参数

### 反向动力学方程
缺实际物理参数