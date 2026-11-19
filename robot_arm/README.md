# PUMA 560 正逆运动学与轨迹演示

对六轴 PUMA 560 机械臂计算正运动学、带关节限位的逆运动学、起止静止的五次关节插值，并生成末端轨迹与 3D 连杆动画。基座原点位于安装面，末端参考点取第六坐标系原点。程序内部使用米和弧度。 [Robotics Toolbox PUMA 560 模型参数](https://raw.githubusercontent.com/petercorke/robotics-toolbox-python/main/src/roboticstoolbox/models/DH/Puma560.py)

## 运行

用 64 位 CPython 3.10–3.14 在本目录安装依赖，执行 `python demo.py` 即可完成数值检查和导出。

```bash
python -m pip install -r requirements.txt
python demo.py
```

默认输入是固定随机种子 `20261119` 生成的测试关节角，以及 `demo.py` 中的两组轨迹关节角。可用 `--output 路径` 指定输出目录。

求解单个自定义目标时，输入 UTF-8 JSON 文件，内容是以基座为参考的 `4 × 4` 齐次变换矩阵。平移列单位为米，左上三阶旋转块须为合法旋转矩阵。`--seed-deg` 接受 6 个初始关节角，单位为度，可用于指定优先搜索的解支路。输出关节角为弧度。

```bash
python demo.py --target unreachable_pose.json
python demo.py --target reachable_pose.json --trace
```

示例的平移是 `[2, 0, 1] m`，输出包含 `"status": "unreachable"`，命令退出码为 `2`。达到目标时返回 `converged` 或 `singular`，后者表示找到的解处于或接近雅可比降秩位形。`not_converged` 表示有限次搜索未找到满足容差的限位内解，`invalid` 表示矩阵或初值无效。

## 正运动学如何对照

`dh_transform` 按标准 DH 构造单节变换 `Rz(qᵢ) Tz(dᵢ) Tx(aᵢ) Rx(αᵢ)`，`forward_frames` 从基座累乘至末端。`demo.py` 的 `compare_forward` 用同一组关节角分别调用本地 `forward(q)` 和已安装的 `Puma560().fkine(q)`。两套实现共享公开的 PUMA 560 模型参数，各自计算变换。对照报告记录了工具箱版本、类、方法和两组 XYZ 值。 [本地实现](kinematics.py) · [对照入口](demo.py) · [工具箱源码](https://github.com/petercorke/robotics-toolbox-python/blob/main/src/roboticstoolbox/robot/DHRobot.py)

Python 3.13 的 10 组样本中，两边的位置浮点值恰好相同。矩阵公式及累乘顺序相同，出现 `0.0 m` 可以成立。回归测试只把本地 DH 表的底座高度 `d₁` 增加 `1 mm`，确认工具箱坐标不变、对照误差变为 `0.001 m`。 `python test_fk_reference.py -v` 可单独复跑。 [回归测试](test_fk_reference.py)

## 数值逆解如何收敛

1. **定义误差。** 给定关节角 `q`，先用正运动学得到 `p(q)` 和 `R(q)`。残差由 3 个位置分量 `p(q) − p*` 和 9 个旋转矩阵分量 `0.30 × vec(R(q) − R*)` 组成。`0.30 m` 为旋转项的权重，对应代码中的 `ROTATION_WEIGHT_M`。求解器最小化 `½‖e(q)‖²`。
2. **计算更新。** 在当前点用 `e(q + Δq) ≈ e(q) + J(q)Δq` 预测一步。转动关节的位置雅可比列为关节轴与指向末端的矢量的叉积，旋转部分对旋转矩阵的每个轴做同样的角速度叉积。SciPy `least_squares(method="trf")` 用这个解析雅可比，在关节上下限和局部信赖域内尝试更新，并根据代价的实际下降调整下一步的范围。 [SciPy TRF 说明](https://docs.scipy.org/doc/scipy-1.15.3/reference/generated/scipy.optimize.least_squares.html)
3. **判断是否达到目标。** SciPy 的 `ftol`、`xtol`、`gtol` 与 `max_nfev` 决定一次优化何时停止。应用再用正运动学验算位置误差 `< 1e-4 m` 和相对旋转角 `< 1e-4 rad`，两项都达标才接受。初值的结果不达标时换起点，目标生成时使用的关节角不参与初始化。检测到加权几何雅可比的最小、最大奇异值之比小于 `1e-5` 时，返回 `singular` 提示。

示例 `reachable_pose.json` 对应验收的第 1 个目标。Python 3.13 的记录显示同一目标在两个初值下的不同结果。

| 尝试 | 代价变化 | 终止条件 | 验收 |
|---|---:|---|---|
| 1 | `0.829 → 0.130` | `ftol`，代价下降已很小 | 位置仍差 `0.377 m`，换初值 |
| 2 | `0.931 → 5.04e-32` | `gtol`，一阶最优性达到阈值 | 位置和姿态都达标 |

`--trace` 输出各次尝试的起点、残差调用次数、终止原因，以及每次雅可比求值点上的代价和位姿误差。可以给同一目标加 `--seed-deg 0 100 -135 0 -45 0`，观察直接从第二个初值出发的结果。20 个测试位姿全部收敛，`1e-15 m` 量级表示理想几何模型之间的数值残差。实物定位精度需要由标定和测量验证。 [完整收敛数据](../results/robot_arm/validation.json)

## 五次轨迹为什么能平滑起停

`quintic_trajectory` 用归一化时间 `s=t/T` 和多项式 `10s³−15s⁴+6s⁵` 插值。多项式的一阶、二阶导数在两个端点都为零，与前后静止段拼接时，关节位置、速度和加速度连续。末端线速度由 `Jv(q)·q̇` 计算，起止时 `J̇v(q)·q̇` 和 `Jv(q)·q̈` 均为零。用 `demo.py` 的 `trajectory_data` 可沿着关节插值、末端位置、末端速度一路读到 `export_gif` 的每帧绘制。 [轨迹实现](demo.py) · [轨迹数据](../results/robot_arm/trajectory.csv)

## 示例输出

| 文件 | 内容 |
|---|---|
| `trajectory.gif` | 4 秒轨迹，101 帧、25 fps，含末端路径、起止位置与随动工具坐标轴 |
| `trajectory.png` | 六关节角度与角速度、末端 XYZ 位置与线速度，300 dpi |
| `trajectory.csv` | 各时刻关节角、关节角速度、关节角加速度、末端位置与末端线速度 |
| `validation.json` | 10 组 FK、20 组 IK、第一组 IK 的收敛过程、异常位姿和轨迹端点 |

本次结果为 FK 最大位置误差 `0.0 m`，IK 成功 `20/20`，以工具箱回算的最大位置误差 `4.11e-15 m`。 [查看曲线](../results/robot_arm/trajectory.png) · [播放动画](../results/robot_arm/trajectory.gif) · [查看验收数据](../results/robot_arm/validation.json)
