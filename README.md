# Python 工程练习与验收

工程包含三个可独立运行的作品。交付包中的 `results/` 是从命令行实际生成的示例结果。

| 作品 | 运行入口 | 使用说明 | 示例结果 |
|---|---|---|---|
| PUMA 560 正逆运动学、轨迹与 3D 动画 | `robot_arm/demo.py` | [机械臂 README](robot_arm/README.md) | [轨迹曲线](results/robot_arm/trajectory.png) · [GIF](results/robot_arm/trajectory.gif) · [数值报告](results/robot_arm/validation.json) |
| CSV/Excel 批量清洗和出图 | `data_plot/plot_data.py` | [数据工具 README](data_plot/README.md) | [传感器温度图](results/data_plot/sensors_csv/01_temperature.png) · [清洗报告](results/data_plot/sensors_csv/cleaning.json) |
| 三组 PID 参数阶跃对比 | `pid_sim/simulate_pid.py` | [PID README](pid_sim/README.md) | [响应曲线](results/pid_sim/step_response.png) · [指标](results/pid_sim/metrics.csv) |

## 安装与验收

使用 64 位 Python 3.13。解压后进入本 `README.md` 所在目录。Windows PowerShell 可直接运行以下命令，无需激活环境。

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe verify.py --output rerun
```

macOS 或 Linux 可将第一条改为 `python3.13 -m venv .venv`，后两条使用 `./.venv/bin/python`。本交付在 Windows 64 位、CPython 3.13 上用新建环境完成安装和运行验证。

`verify.py` 会调用三个作品的命令行入口。`--output` 接受新目录或空目录，以保留既有结果。省略 `--output` 会在临时目录运行验收。故意输入的坏文件会出现 `[ERROR]`，验收脚本检查报错内容和退出状态。最终出现 `All acceptance checks passed.` 即表示通过。各子目录也提供独立的 `requirements.txt`。

## 本次验收记录

| 检查项 | 结果 |
|---|---|
| PUMA 560 正运动学 | 10 组位置最大误差 `0.0 m`，阈值 `< 1e-6 m` |
| PUMA 560 逆运动学 | `20/20` 个完整位姿收敛，最大位置误差 `4.11e-15 m`，最大姿态误差 `9.35e-15 rad`，两种不可达位姿和一组腕部奇异位形均有提示 |
| 五次轨迹与 GIF | 起止点关节及末端线速度和加速度均为 `0`，`740 × 620`，`101` 帧，每帧 `40 ms` |
| 数据工具 | 三份合成数据全部通过，输出 6 张 300 dpi PNG 和对应的矢量 PDF，缺失、乱码、空文件和坏数值均有诊断，输入文件 SHA-256 校验一致 |
| PID | 三组调节时间与 `control` 一致，最大超调量差 `2.54e-12` 个百分点以内 |

详细状态见 [总体验收记录](results/acceptance.json)。
