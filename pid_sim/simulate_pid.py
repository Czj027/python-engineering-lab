"""Compare three PID controllers on one continuous-time second-order plant."""

import argparse
import csv
from dataclasses import asdict, dataclass
from importlib.metadata import version
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
from matplotlib import font_manager
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal


DT = 0.002
HORIZON = 30.0
BAND = 0.02
COLORS = ["#1667a5", "#cc5529", "#008473"]
LINE_STYLES = ["-", "--", "-."]


@dataclass(frozen=True)
class PID:
    name: str
    kp: float
    ki: float
    kd: float


def read_configs(path):
    source = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(source, list) or len(source) != 3:
        raise ValueError("配置必须是包含恰好 3 组 PID 参数的 JSON 数组")
    configs = []
    for item in source:
        if not isinstance(item, dict) or set(item) != {"name", "kp", "ki", "kd"}:
            raise ValueError("每组配置必须包含 name、kp、ki、kd")
        name = item["name"]
        if (not isinstance(name, str) or not name.strip() or any(c in name for c in "\r\n\t")
                or len(name) > 32):
            raise ValueError("name 必须为非空单行文字，长度不超过 32 个字符")
        gains = [item[key] for key in ["kp", "ki", "kd"]]
        if any(type(value) not in (int, float) or not np.isfinite(float(value)) for value in gains):
            raise ValueError(f"{name} 的 PID 参数必须为有限实数")
        kp, ki, kd = map(float, gains)
        if kp < 0 or ki <= 0 or kd < 0:
            raise ValueError(f"{name} 的参数需要满足 Kp ≥ 0、Ki > 0、Kd ≥ 0")
        configs.append(PID(name.strip(), kp, ki, kd))
    if len({item.name for item in configs}) != len(configs):
        raise ValueError("三组配置需要使用不同名称")
    return configs


def closed_loop(pid):
    # T(s) = C(s)G(s) / (1 + C(s)G(s)), with G(s) = 1 / (s+1)^2.
    return [pid.kd, pid.kp, pid.ki], [1, 2 + pid.kd, 1 + pid.kp, pid.ki]


def step_metrics(time, response, final_value):
    peak_index = int(np.argmax(response))
    maximum = float(response[peak_index])
    overshoot = max(0.0, (maximum - final_value) / final_value * 100)
    outside = np.flatnonzero(np.abs(response / final_value - 1) >= BAND)
    if outside.size and outside[-1] == len(time) - 1:
        raise ValueError("响应到仿真末尾仍未进入 2% 调节带")
    settled = int(outside[-1]) + 1 if outside.size else 0
    return {"overshoot_percent": overshoot, "settling_time_s": float(time[settled]),
            "peak": maximum, "peak_time_s": float(time[peak_index])}


def simulate(pids):
    import control

    systems = [closed_loop(pid) for pid in pids]
    poles = [np.roots(denominator) for _, denominator in systems]
    for pid, roots in zip(pids, poles):
        if np.max(roots.real) >= 0:
            raise ValueError(f"{pid.name} 闭环不稳定，超调量和调节时间无法按稳态定义计算")
    slowest_decay = min(float(-np.max(roots.real)) for roots in poles)
    duration = max(HORIZON, 12 / slowest_decay)
    steps = int(np.ceil(duration / DT))
    if steps > 200_000:
        raise ValueError("所选参数的闭环衰减太慢，超过本演示的 400 s 仿真上限")
    time = np.arange(steps + 1) * DT
    responses = []
    cases = []
    s = control.tf("s")
    plant = 1 / (s + 1) ** 2
    for pid, (numerator, denominator), roots in zip(pids, systems, poles):
        _, response = signal.step((numerator, denominator), T=time)
        final_value = numerator[-1] / denominator[-1]
        metrics = step_metrics(time, response, final_value)
        reference = control.feedback((pid.kp * s + pid.ki + pid.kd * s**2) / s * plant, 1)
        reference_response = np.asarray(control.step_response(reference, timepts=time).outputs)
        check = control.step_info(reference, timepts=time, SettlingTimeThreshold=BAND)
        comparison = {"control_overshoot_percent": float(check["Overshoot"]),
                      "control_settling_time_s": float(check["SettlingTime"]),
                      "max_response_difference": float(np.max(np.abs(response - reference_response))),
                      "overshoot_difference_pp": abs(metrics["overshoot_percent"] - check["Overshoot"]),
                      "settling_time_difference_s": abs(metrics["settling_time_s"] - check["SettlingTime"])}
        cases.append({**asdict(pid), "steady_state": final_value, "numerator": numerator,
                      "denominator": denominator,
                      "poles": [[float(pole.real), float(pole.imag)] for pole in roots],
                      **metrics, **comparison})
        responses.append(response)
    report = {"plant": "1/(s+1)^2", "reference_step": 1, "time_step_s": DT,
              "horizon_s": float(time[-1]), "settling_band_fraction": BAND,
              "control_version": version("control"), "cases": cases}
    return time, responses, report


def export(folder, time, responses, report):
    folder.mkdir(parents=True, exist_ok=True)
    cases = report["cases"]
    with (folder / "metrics.csv").open("w", encoding="utf-8-sig", newline="") as file:
        keys = ["name", "kp", "ki", "kd", "steady_state", "overshoot_percent", "settling_time_s",
                "peak", "peak_time_s", "control_overshoot_percent", "control_settling_time_s",
                "max_response_difference", "overshoot_difference_pp", "settling_time_difference_s"]
        writer = csv.DictWriter(file, keys, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(cases)
    with (folder / "step_response.csv").open("w", encoding="utf-8-sig", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["time_s", *[case["name"] for case in cases]])
        writer.writerows(np.column_stack((time, *responses)))
    (folder / "validation.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                                             encoding="utf-8")

    installed = set(font_manager.get_font_names())
    fonts = [name for name in ["DejaVu Sans", "Microsoft YaHei", "Noto Sans CJK SC"] if name in installed]
    with plt.rc_context({"font.family": fonts,
                         "font.size": 10, "pdf.fonttype": 42, "text.parse_math": False}):
        fig, axis = plt.subplots(figsize=(8.2, 4.8), layout="constrained")
        axis.set_title("PID tuning | Unit step response", loc="left", fontsize=15, fontweight="bold", pad=13)
        axis.axhspan(1 - BAND, 1 + BAND, color="#bed8c8", alpha=0.55, label="±2% settling band")
        axis.axhline(1, color="#576572", linewidth=0.9)
        for case, response, color, linestyle in zip(cases, responses, COLORS, LINE_STYLES):
            axis.plot(time, response, color=color, linestyle=linestyle, linewidth=1.85,
                      label=case["name"])
            settling = case["settling_time_s"]
            axis.scatter(settling, np.interp(settling, time, response), s=32, color=color,
                         edgecolors="white", linewidths=0.7, zorder=4)
        last_peak = max((case["peak_time_s"] for case in cases
                         if case["overshoot_percent"] > 1e-3), default=0)
        last_settling = max(case["settling_time_s"] for case in cases)
        axis.set(xlim=(0, min(float(time[-1]), max(6, last_settling * 1.25, last_peak * 1.12))),
                 xlabel="Time (s)", ylabel="Output (dimensionless)")
        axis.spines[["top", "right"]].set_visible(False)
        axis.grid(alpha=0.24)
        axis.set_axisbelow(True)
        axis.legend(loc="lower right", frameon=False, fontsize=9)
        fig.savefig(folder / "step_response.png", dpi=300)
        fig.savefig(folder / "step_response.pdf", dpi=300)
        plt.close(fig)


def main():
    root = Path(__file__).parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=root / "example_gains.json")
    parser.add_argument("--output", type=Path, default=root / "results")
    args = parser.parse_args()
    try:
        pids = read_configs(args.config)
        time, responses, report = simulate(pids)
        export(args.output, time, responses, report)
    except (OSError, OverflowError, ValueError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2
    for case in report["cases"]:
        print(f"{case['name']}: overshoot {case['overshoot_percent']:.3f}%, "
              f"Ts(2%) {case['settling_time_s']:.3f} s, "
              f"max curve delta vs control {case['max_response_difference']:.2e}")
    print(f"Saved: {args.output}")
    passed = all(case["max_response_difference"] < 1e-9 and case["overshoot_difference_pp"] < 1e-7
                 and case["settling_time_difference_s"] <= DT for case in report["cases"])
    return 0 if passed else 1


if __name__ == "__main__":
    sys.stdout.reconfigure(errors="backslashreplace")
    raise SystemExit(main())
