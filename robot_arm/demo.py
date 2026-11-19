"""Validate PUMA 560 kinematics and export a complete rest-to-rest move."""

import argparse
import csv
from dataclasses import asdict
from importlib.metadata import version
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
from matplotlib.animation import FuncAnimation, PillowWriter
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np

from kinematics import LOWER, UPPER, forward, forward_frames, geometric_jacobian
from kinematics import inverse, pose_errors, quintic_trajectory


SEED = 20261119
DURATION_S = 4.0
FPS = 25
COLORS = ["#1565a8", "#c44e21", "#00836e", "#7855a5", "#925918", "#ab437b"]


def validate_kinematics():
    from roboticstoolbox.models.DH import Puma560

    reference = Puma560()
    rng = np.random.default_rng(SEED)
    fk_cases = []
    for index, q in enumerate(rng.uniform(LOWER, UPPER, (10, 6)), 1):
        toolbox = reference.fkine(q).A
        actual = forward(q)
        linear, angular = pose_errors(actual, toolbox)
        fk_cases.append({"case": index, "q_rad": q.tolist(), "actual_xyz_m": actual[:3, 3].tolist(),
                         "toolbox_xyz_m": toolbox[:3, 3].tolist(), "position_error_m": linear,
                         "orientation_error_rad": angular})

    ik_cases = []
    for index, q in enumerate(rng.uniform(LOWER, UPPER, (20, 6)), 1):
        target = reference.fkine(q).A
        result = inverse(target)
        row = {"case": index, "target": target.tolist(), "target_source_q_rad": q.tolist(),
               **asdict(result)}
        if result.q_rad is not None:
            linear, angular = pose_errors(reference.fkine(result.q_rad).A, target)
            row.update(toolbox_position_error_m=linear, toolbox_orientation_error_rad=angular)
        ik_cases.append(row)

    singular_q = np.deg2rad([15, 38, -65, 12, 0, -20])
    far = np.eye(4)
    far[:3, 3] = [2, 0, 1]
    inside = np.eye(4)
    inside[:3, 3] = [0, 0, 0.8]
    cases = {
        "unreachable": asdict(inverse(far)),
        "unreachable_inside_shoulder_offset": asdict(inverse(inside)),
        "singular": asdict(inverse(reference.fkine(singular_q).A, singular_q)),
        "invalid_pose": asdict(inverse(np.zeros((4, 4)))),
    }
    success = sum(row["status"] == "converged" and row["toolbox_position_error_m"] < 1e-4
                  and row["toolbox_orientation_error_rad"] < 1e-4 for row in ik_cases)
    return {"robot": "PUMA 560", "roboticstoolbox_version": version("roboticstoolbox-python"),
            "random_seed": SEED, "fk": fk_cases, "ik": ik_cases, "ik_successes": success, "cases": cases}


def trajectory_data():
    start = np.deg2rad([-48, 14, -55, -30, 65, -12])
    stop = np.deg2rad([43, 55, -79, 42, 24, 55])
    time = np.linspace(0, DURATION_S, round(DURATION_S * FPS) + 1)
    q, qd, qdd = quintic_trajectory(start, stop, time, DURATION_S)
    frames = np.stack([forward_frames(angles) for angles in q])
    jacobians = np.stack([geometric_jacobian(chain) for chain in frames])
    position = frames[:, -1, :3, 3]
    velocity = np.einsum("tij,tj->ti", jacobians[:, :3, :], qd)
    # At both endpoints qd = 0, so Jdot·qd vanishes in the tool acceleration.
    ends = [0, -1]
    end_acceleration = np.einsum("tij,tj->ti", jacobians[ends, :3, :], qdd[ends])
    summary = {"duration_s": DURATION_S, "frames": len(time), "frame_rate_fps": FPS,
               "start_q_rad": start.tolist(), "end_q_rad": stop.tolist(),
               "endpoint_maxima": {"joint_speed_rad_s": float(abs(qd[ends]).max()),
                                   "joint_acceleration_rad_s2": float(abs(qdd[ends]).max()),
                                   "tool_speed_m_s": float(abs(velocity[ends]).max()),
                                   "tool_acceleration_m_s2": float(abs(end_acceleration).max())}}
    return time, q, qd, qdd, frames, position, velocity, summary


def export_curves(folder, time, q, qd, qdd, position, velocity):
    fig, axes = plt.subplots(2, 2, figsize=(10.4, 6.3), sharex=True, layout="constrained")
    fig.suptitle("PUMA 560 | Rest-to-rest joint trajectory", fontsize=16, fontweight="bold")
    plots = [(axes[0, 0], np.rad2deg(q), "Joint angle (deg)", [f"J{i}" for i in range(1, 7)]),
             (axes[1, 0], np.rad2deg(qd), "Joint speed (deg/s)", [f"J{i}" for i in range(1, 7)]),
             (axes[0, 1], position, "Tool position (m)", ["X", "Y", "Z"]),
             (axes[1, 1], velocity, "Tool velocity (m/s)", ["X", "Y", "Z"])]
    for axis, values, label, names in plots:
        for values_i, name, color in zip(values.T, names, COLORS):
            axis.plot(time, values_i, label=name, color=color, linewidth=1.65)
        axis.set_ylabel(label)
        axis.margins(x=0.025)
        axis.grid(alpha=0.25)
        axis.spines[["top", "right"]].set_visible(False)
        axis.legend(loc="best", ncols=3, fontsize=8, frameon=False)
    for axis in axes[1]:
        axis.set_xlabel("Time (s)")
    fig.savefig(folder / "trajectory.png", dpi=300)
    plt.close(fig)

    fieldnames = ["time_s"]
    for prefix, unit in [("q", "rad"), ("qd", "rad_s"), ("qdd", "rad_s2")]:
        fieldnames.extend(f"{prefix}{i}_{unit}" for i in range(1, 7))
    fieldnames += ["x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s"]
    with (folder / "trajectory.csv").open("w", encoding="utf-8", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(fieldnames)
        writer.writerows(np.column_stack((time, q, qd, qdd, position, velocity)))


def export_gif(path, time, frames, position):
    fig = plt.figure(figsize=(7.4, 6.2), facecolor="#ffffff")
    axis = fig.add_subplot(projection="3d", computed_zorder=False)
    fig.subplots_adjust(left=0.015, right=0.95, top=0.88, bottom=0.16)
    fig.text(0.07, 0.94, "PUMA 560 | 3D trajectory", fontsize=17, weight="bold", color="#152b43")
    tool_length = 0.095
    tool_tips = position[:, None, :] + tool_length * frames[:, -1, :3, :3].transpose(0, 2, 1)
    points = np.vstack((frames[:, :, :3, 3].reshape(-1, 3), tool_tips.reshape(-1, 3)))
    low, high = points.min(axis=0) - 0.12, points.max(axis=0) + 0.12
    axis.set(xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)",
             xlim=(low[0], high[0]), ylim=(low[1], high[1]), zlim=(low[2], high[2]))
    axis.set_box_aspect(high - low)
    axis.view_init(elev=23, azim=-54)
    for coordinate in (axis.xaxis, axis.yaxis, axis.zaxis):
        coordinate.set_pane_color((0.95, 0.97, 0.99, 0.75))
        coordinate.set_major_locator(MaxNLocator(nbins=5))
    axis.plot(*position.T, linestyle="--", linewidth=1.5, color="#7590a7", label="Tool path", zorder=1)
    axis.scatter(*position[0], color="#168268", label="Start", s=36, depthshade=False, zorder=3)
    axis.scatter(*position[-1], color="#c44e21", label="Finish", s=38, marker="D", depthshade=False, zorder=3)
    pedestal, = axis.plot([], [], [], linewidth=8, color="#788da2", solid_capstyle="round", zorder=4)
    arm, = axis.plot([], [], [], "o-", linewidth=5, color="#1664a8", markerfacecolor="#f2f7fa",
                    markersize=7, markeredgewidth=1.8, solid_capstyle="round", zorder=5)
    trail, = axis.plot([], [], [], color="#168268", linewidth=2.1, zorder=2)
    handles, names = axis.get_legend_handles_labels()
    fig.legend(handles, names, loc="lower center", bbox_to_anchor=(0.50, 0.030),
               ncols=3, frameon=False, fontsize=10)
    tool_axes = [axis.plot([], [], [], linewidth=2.2, color=color, zorder=6)[0]
                 for color in ["#c44e21", "#168268", "#7855a5"]]
    label = fig.text(0.70, 0.94, "", fontsize=10, color="#486982")

    def update(index):
        origins = frames[index, :, :3, 3]
        pedestal.set_data_3d(*origins[:2].T)
        arm.set_data_3d(*origins[1:5].T)
        trail.set_data_3d(*position[:index + 1].T)
        origin = origins[-1]
        for line, tip in zip(tool_axes, tool_tips[index]):
            line.set_data_3d(*np.vstack((origin, tip)).T)
        label.set_text(f"t = {time[index]:.2f} / {time[-1]:.2f} s")

    animation = FuncAnimation(fig, update, frames=len(time), interval=1000 / FPS, cache_frame_data=False)
    animation.save(path, writer=PillowWriter(fps=FPS), dpi=100)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path(__file__).parent / "results")
    parser.add_argument("--target", type=Path, help="Solve a JSON file containing one 4x4 pose matrix")
    parser.add_argument("--seed-deg", type=float, nargs=6, help="Optional IK initial joint angles in degrees")
    args = parser.parse_args()
    if args.seed_deg is not None and args.target is None:
        parser.error("--seed-deg requires --target")
    if args.target is not None:
        try:
            target = json.loads(args.target.read_text(encoding="utf-8-sig"))
        except (OSError, ValueError) as exc:
            print(f"[ERROR] 无法读取目标 JSON：{exc}", file=sys.stderr)
            return 2
        seed = None if args.seed_deg is None else np.deg2rad(args.seed_deg)
        result = inverse(target, seed)
        print(json.dumps(asdict(result), ensure_ascii=False, indent=2))
        return 0 if result.status in {"converged", "singular"} else 2

    args.output.mkdir(parents=True, exist_ok=True)
    report = validate_kinematics()
    time, q, qd, qdd, frames, position, velocity, report["trajectory"] = trajectory_data()
    max_fk = max(row["position_error_m"] for row in report["fk"])
    print(f"FK: 10 poses, max position error {max_fk:.3e} m.", flush=True)
    print(f"IK: {report['ik_successes']}/20 regular, converged, in-limit full poses.", flush=True)
    for name, case in report["cases"].items():
        print(f"{name}: {case['status']} — {case['message']}", flush=True)
    export_curves(args.output, time, q, qd, qdd, position, velocity)
    export_gif(args.output / "trajectory.gif", time, frames, position)
    (args.output / "validation.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                                                 encoding="utf-8")
    print(f"Saved: {args.output}")
    flagged = [case["status"] for case in report["cases"].values()]
    passed = max_fk < 1e-6 and report["ik_successes"] >= 18 and flagged == [
        "unreachable", "unreachable", "singular", "invalid"]
    return 0 if passed else 1


if __name__ == "__main__":
    sys.stdout.reconfigure(errors="backslashreplace")
    raise SystemExit(main())
