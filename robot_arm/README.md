# PUMA 560 正逆运动学与轨迹演示

对六轴 PUMA 560 机械臂计算正运动学、带关节限位的逆运动学、起止静止的五次关节插值，并生成末端轨迹与 3D 连杆动画。基座原点位于安装面，末端参考点取第六坐标系原点。程序内部使用米和弧度。 [Robotics Toolbox PUMA 560 模型参数](https://raw.githubusercontent.com/petercorke/robotics-toolbox-python/main/src/roboticstoolbox/models/DH/Puma560.py)

## 运行

用 Python 3.13 在本目录安装一次依赖，之后执行 `python demo.py` 即可完成数值检查和导出。

```bash
python -m pip install -r requirements.txt
python demo.py
```

默认输入是固定随机种子 `20261119` 生成的测试关节角，以及 `demo.py` 中的两组轨迹关节角。可用 `--output 路径` 指定输出目录。

求解单个自定义目标时，输入 UTF-8 JSON 文件，内容是以基座为参考的 `4 × 4` 齐次变换矩阵。平移列单位为米，左上三阶旋转块须为合法旋转矩阵。`--seed-deg` 接受 6 个初始关节角，单位为度，可用于指定优先搜索的解支路。输出关节角为弧度。

```bash
python demo.py --target unreachable_pose.json
```

示例的平移是 `[2, 0, 1] m`，输出包含 `"status": "unreachable"`，命令退出码为 `2`。达到目标时返回 `converged` 或 `singular`，后者表示找到的解处于或接近雅可比降秩位形。`not_converged` 表示有限次搜索未找到满足容差的限位内解，`invalid` 表示矩阵或初值无效。

## 方法

1. `kinematics.py` 用标准 DH 变换逐节相乘求取末端位姿。随机关节角覆盖模型的各关节限位范围，每组分别用本实现和 `roboticstoolbox` 计算。
2. 逆解用带限位的多起点最小二乘，同时最小化位置和旋转矩阵残差，使用解析雅可比加快求解。返回前检查位置误差 `< 1e-4 m` 和相对旋转角 `< 1e-4 rad`。测试的 20 个目标由工具箱正运动学生成，生成目标的关节角未作为求解初值。奇异性由加权几何雅可比的最小、最大奇异值之比判断，阈值为 `1e-5`。数值求解可能存在多个合法分支。
3. 用归一化时间 `s=t/T` 和多项式 `10s³−15s⁴+6s⁵` 插值。多项式的一阶、二阶导数在两个端点都为零，因此与前后静止段拼接时，关节位置、速度和加速度连续。末端线速度由 `Jv(q)·q̇` 计算，起止时 `J̇v(q)·q̇` 和 `Jv(q)·q̈` 均为零。

这是关节空间内的运动学演示。末端路径由关节插值决定，模型未包含碰撞检查、动力学或实物执行。

## 示例输出

| 文件 | 内容 |
|---|---|
| `trajectory.gif` | 4 秒轨迹，101 帧、25 fps，含末端路径、起止位置与随动工具坐标轴 |
| `trajectory.png` | 六关节角度与角速度、末端 XYZ 位置与线速度，300 dpi |
| `trajectory.csv` | 各时刻关节角、关节角速度、关节角加速度、末端位置与末端线速度 |
| `validation.json` | 10 组 FK、20 组 IK、异常位姿与轨迹端点的逐项结果 |

本次结果为 FK 最大位置误差 `0.0 m`，IK 成功 `20/20`，以工具箱回算的最大位置误差 `4.11e-15 m`。 [查看曲线](../results/robot_arm/trajectory.png) · [播放动画](../results/robot_arm/trajectory.gif) · [查看验收数据](../results/robot_arm/validation.json)
