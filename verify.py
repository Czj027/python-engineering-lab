"""Run the three public CLIs and check the delivered acceptance cases."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory


ROOT = Path(__file__).resolve().parent
DATA = ROOT / "data_plot" / "examples"


def require(condition, detail):
    if not condition:
        raise RuntimeError(detail)


def run(script, *args, code=0, contains=None):
    command = [sys.executable, str(ROOT / script), *map(str, args)]
    result = subprocess.run(command, cwd=ROOT, capture_output=True,
                            encoding="utf-8", env={**os.environ, "PYTHONIOENCODING": "utf-8"})
    output = result.stdout + result.stderr
    print(output.strip(), flush=True)
    require(result.returncode == code, f"{script}: expected exit {code}, got {result.returncode}")
    require("Traceback (most recent call last)" not in output, f"{script}: unhandled exception")
    if contains:
        require(contains in output, f"{script}: missing diagnostic {contains!r}")
    return output


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def png_dpi(path):
    from PIL import Image

    with Image.open(path) as img:
        img.load()
        require(all(abs(dpi - 300) < 0.1 for dpi in img.info["dpi"]), f"Wrong DPI: {path}")


def robot_checks(folder):
    from PIL import Image

    run("robot_arm/test_fk_reference.py")
    run("robot_arm/demo.py", "--output", folder)
    run("robot_arm/demo.py", "--target", ROOT / "robot_arm" / "unreachable_pose.json",
        code=2, contains='"status": "unreachable"')
    report = read_json(folder / "validation.json")
    max_fk = max(row["position_error_m"] for row in report["fk"])
    require(len(report["fk"]) == 10 and max_fk < 1e-6,
            "FK: 10 toolbox comparisons must meet 1e-6 m")
    converged = [row for row in report["ik"] if row["status"] == "converged" and
                 row["toolbox_position_error_m"] < 1e-4 and row["toolbox_orientation_error_rad"] < 1e-4]
    require(len(report["ik"]) == 20 and len(converged) >= 18, "IK: at least 18/20 full poses must pass")
    attempts = report["ik"][0]["trace"]
    require(attempts, "First IK case must record the numerical solve")
    final_attempt = attempts[-1]
    evaluations = final_attempt["jacobian_evaluations"]
    require(len(evaluations) > 1 and final_attempt["final_cost"] < evaluations[0]["cost"],
            "IK trace: the last attempt should show progress from its starting cost")
    require(report["cases"]["unreachable"]["status"] == "unreachable", "Missing unreachable status")
    require(report["cases"]["singular"]["status"] == "singular", "Missing singular status")
    for magnitude in report["trajectory"]["endpoint_maxima"].values():
        require(magnitude <= 1e-12, "Trajectory derivatives must vanish at the endpoints")
    png_dpi(folder / "trajectory.png")
    with Image.open(folder / "trajectory.gif") as gif:
        count = gif.n_frames
        require(count == 101, f"Expected 101 GIF frames, got {count}")
        first = gif.convert("RGB").tobytes()
        durations = []
        for frame in range(count):
            gif.seek(frame)
            durations.append(gif.info["duration"])
        require(first != gif.convert("RGB").tobytes(), "GIF should reach a distinct end pose")
        require(set(durations) == {40}, "GIF should play at 25 frames per second")
    return {"fk_cases": len(report["fk"]), "fk_max_position_error_m": max_fk,
            "fk_model_perturbation_checked": True, "ik_convergence_trace_checked": True,
            "ik_cases": len(report["ik"]), "ik_converged": len(converged),
            "ik_max_position_error_m": max(row["toolbox_position_error_m"] for row in converged),
            "ik_max_orientation_error_rad": max(row["toolbox_orientation_error_rad"] for row in converged),
            "gif_frames": count, "gif_frame_ms": 40}


def data_checks(folder):
    successful_inputs = [DATA / "sensors.csv", DATA / "drive.csv", DATA / "calibration.xlsx"]
    bad = DATA / "invalid"
    sources = successful_inputs + sorted(bad.glob("*.csv"))
    before = {file: hashlib.sha256(file.read_bytes()).digest() for file in sources}
    run("data_plot/plot_data.py", successful_inputs[0], successful_inputs[2], "--output", folder)
    run("data_plot/plot_data.py", successful_inputs[1], "--decimal", ",", "--output", folder)
    verified = []
    for stem, series_count, rows in [("sensors_csv", 2, 7), ("drive_csv", 2, 7),
                                    ("calibration_xlsx", 2, 7)]:
        batch = folder / stem
        report = read_json(batch / "cleaning.json")
        require(report["rows_written"] == rows, f"Unexpected cleaned row count for {stem}")
        require(len(report["series"]) == series_count, f"Wrong series count for {stem}")
        require((batch / "cleaned.csv").is_file(), f"No cleaned CSV for {stem}")
        for series in report["series"]:
            require(series["unit"] and report["x"]["unit"], "An axis is missing its unit")
        images = sorted(batch.glob("*.png"))
        require(len(images) == series_count, f"Wrong PNG count for {stem}")
        for img in images:
            png_dpi(img)
            require(img.with_suffix(".pdf").read_bytes().startswith(b"%PDF-"), f"No PDF for {img}")
        if stem == "sensors_csv":
            require(report["duplicate_rows"] > 0 and sum(s["missing"] for s in report["series"]) > 0,
                    "Missing values and duplicate removal should be recorded")
        verified.append(stem)
    issues = {"empty.csv": "空文件", "garbled.csv": "乱码", "bad_values.csv": "非数值",
              "all_missing.csv": "有效数据点不足"}
    for name, diagnostic in issues.items():
        run("data_plot/plot_data.py", bad / name, "--output", folder, code=1, contains=diagnostic)
    run("data_plot/plot_data.py", successful_inputs[0], "--strict-missing", "--output", folder,
        code=1, contains="缺失值")
    require(all(hashlib.sha256(file.read_bytes()).digest() == old for file, old in before.items()),
            "Data CLI changed an original file")
    return {"successful_inputs": verified, "invalid_inputs": list(issues),
            "strict_missing_verified": True, "source_hashes_unchanged": True, "png_dpi": 300}


def pid_checks(folder):
    run("pid_sim/simulate_pid.py", "--output", folder)
    report = read_json(folder / "validation.json")
    require(len(report["cases"]) == 3, "PID must compare three controllers")
    for case in report["cases"]:
        require(case["max_response_difference"] < 1e-9, "Step curves disagree with python-control")
        require(case["overshoot_difference_pp"] < 1e-7, "Overshoot disagrees with python-control")
        require(case["settling_time_difference_s"] <= report["time_step_s"],
                "Settling time differs by more than one sample")
    png_dpi(folder / "step_response.png")
    return {"controllers": len(report["cases"]), "settling_band_percent": 2,
            "max_overshoot_difference_pp": max(case["overshoot_difference_pp"] for case in report["cases"]),
            "max_settling_time_difference_s": max(case["settling_time_difference_s"] for case in report["cases"])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="Save artifacts to a new or empty directory")
    args = parser.parse_args()
    with TemporaryDirectory(prefix="engineering-acceptance-") as temporary:
        folder = args.output.resolve() if args.output else Path(temporary)
        if folder.exists() and any(folder.iterdir()):
            parser.error("--output must be a new or empty directory")
        folder.mkdir(parents=True, exist_ok=True)
        try:
            results = {"robot_arm": robot_checks(folder / "robot_arm"),
                       "data_plot": data_checks(folder / "data_plot"),
                       "pid_sim": pid_checks(folder / "pid_sim")}
        except RuntimeError as exc:
            print(f"Acceptance failed: {exc}", file=sys.stderr)
            return 1
        (folder / "acceptance.json").write_text(json.dumps(results, ensure_ascii=False, indent=2) + "\n",
                                                encoding="utf-8")
    print("All acceptance checks passed.")
    return 0


if __name__ == "__main__":
    sys.stdout.reconfigure(errors="backslashreplace")
    raise SystemExit(main())
