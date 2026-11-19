# 三组 PID 参数的阶跃响应仿真

用同一二阶对象和单位负反馈比较三组 PID 增益。程序生成叠加响应曲线、记录峰值与 2% 调节时间，并用 `python-control` 对照每条响应和两个关键指标。

## 安装与运行

在本目录使用 64 位 CPython 3.10–3.14 安装依赖，再运行默认示例。

```bash
python -m pip install -r requirements.txt
python simulate_pid.py
```

默认从 `example_gains.json` 读入三组参数，输出到 `results/`。复制配置并修改参数后，可以用 `python simulate_pid.py --config my_gains.json --output trial` 做自己的对比。输入是恰好三项的 JSON 数组。每项含唯一的 `name` 与 `kp`、`ki`、`kd`，满足 `kp ≥ 0`、`ki > 0`、`kd ≥ 0`。

```json
[
  {"name": "Conservative", "kp": 0.8, "ki": 0.65, "kd": 0.35},
  {"name": "Aggressive", "kp": 6.0, "ki": 8.0, "kd": 0.2},
  {"name": "Balanced", "kp": 4.0, "ki": 2.5, "kd": 1.2}
]
```

## 仿真模型与指标

对象为 `G(s) = 1/(s+1)²`，连续 PID 为 `C(s) = Kp + Ki/s + Kd·s`。对单位阶跃，闭环传递函数为

```text
                      Kd·s² + Kp·s + Ki
T(s) = -------------------------------------------------
        s³ + (2 + Kd)·s² + (1 + Kp)·s + Ki
```

程序检查闭环极点实部为负，并用 SciPy 计算响应。超调量为 `max(0, (峰值 − 稳态值) / 稳态值) × 100%`，稳态值由 `T(0)=1` 得到。2% 调节时间取最后一个越出 ±2% 带的采样点之后的第一点。计算步长为 `0.002 s`，默认示例采用 `30 s` 的完整响应来核算指标，图显示主要动态过程。参考结果由 `control.feedback` 建模，以 `control.step_response` 和 `control.step_info` 独立核对。 [python-control 指标说明](https://python-control.readthedocs.io/en/0.10.2/generated/control.step_info.html)

模型采用理想连续 PID，适合比较参数对线性响应的影响。仿真未加入执行器饱和、微分滤波或数字控制器的采样保持。

## 示例输出

| 参数组 | 超调量 | 2% 调节时间 |
|---|---:|---:|
| Conservative | 4.079% | 8.120 s |
| Aggressive | 56.331% | 9.318 s |
| Balanced | 7.447% | 2.990 s |

默认三组参数下，Balanced 的调节时间最短，Conservative 的超调量最小。Aggressive 的峰值出现早，但振荡加长了调节时间。三组与 `control` 的调节时间差均为 `0 s`，整条响应的最大逐点差为 `5.19e-14` 以内。

| 文件 | 内容 |
|---|---|
| `step_response.png`、`step_response.pdf` | 三组曲线、2% 阴影带、各组调节点 |
| `metrics.csv` | 增益、峰值、超调量、调节时间及 `control` 对照值 |
| `step_response.csv` | 完整仿真时间序列 |
| `validation.json` | 闭环系数、极点、采样条件与逐项误差 |

[查看响应图](../results/pid_sim/step_response.png) · [查看指标 CSV](../results/pid_sim/metrics.csv) · [查看对照明细](../results/pid_sim/validation.json)

## 从哪读代码

在 `closed_loop` 对照上面的传递函数系数。`simulate` 用 SciPy 模拟，并在 `control` 中重新搭建单位负反馈作为参照。`step_metrics` 从时间序列找峰值和最后一个越界点。可以在 `step_response.csv` 中定位激进组最后一个超过 2% 调节带的采样点，再核对下一点是否为 `9.318 s`。 [仿真代码](simulate_pid.py) · [响应原始数据](../results/pid_sim/step_response.csv)
