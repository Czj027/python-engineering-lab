"""PUMA 560 standard-DH kinematics, with radians and metres throughout."""

from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


# Columns: a [m], d [m], alpha [rad]. Frame 0 is at the mounting surface.
DH = np.array([
    [0.0, 26.45 * 0.0254, np.pi / 2],
    [0.4318, 0.0, 0.0],
    [0.0203, 0.15005, -np.pi / 2],
    [0.0, 0.4318, np.pi / 2],
    [0.0, 0.0, -np.pi / 2],
    [0.0, 0.0, 0.0],
])
LOWER, UPPER = np.deg2rad([[-160, -45, -225, -110, -100, -266],
                           [160, 225, 45, 170, 100, 266]])
POSITION_TOL = 1e-4
ORIENTATION_TOL = 1e-4
ROTATION_WEIGHT_M = 0.30
SINGULAR_RATIO = 1e-5


def dh_transform(theta, a, d, alpha):
    cosine, sine = np.cos(theta), np.sin(theta)
    twist_cosine, twist_sine = np.cos(alpha), np.sin(alpha)
    return np.array([
        [cosine, -sine * twist_cosine, sine * twist_sine, a * cosine],
        [sine, cosine * twist_cosine, -cosine * twist_sine, a * sine],
        [0, twist_sine, twist_cosine, d],
        [0, 0, 0, 1],
    ])


def forward_frames(q):
    """Return base and all six frames as an array with shape (7, 4, 4)."""
    q = np.asarray(q, dtype=float)
    if q.shape != (6,) or not np.isfinite(q).all():
        raise ValueError("需要 6 个有限关节角（rad）")
    frames = [np.eye(4)]
    for theta, (a, d, alpha) in zip(q, DH):
        frames.append(frames[-1] @ dh_transform(theta, a, d, alpha))
    return np.stack(frames)


def forward(q):
    return forward_frames(q)[-1]


def geometric_jacobian(frames):
    axes = frames[:-1, :3, 2]
    moment_arms = frames[-1, :3, 3] - frames[:-1, :3, 3]
    return np.vstack((np.cross(axes, moment_arms).T, axes.T))


def pose_errors(actual, target):
    position = np.linalg.norm(actual[:3, 3] - target[:3, 3])
    angle = Rotation.from_matrix(target[:3, :3].T @ actual[:3, :3]).magnitude()
    return float(position), float(angle)


@dataclass(frozen=True)
class IKResult:
    status: str
    message: str
    q_rad: list[float] | None = None
    position_error_m: float | None = None
    orientation_error_rad: float | None = None
    singular_value_ratio: float | None = None
    attempts: int = 0


def inverse(target, seed_q=None):
    """Solve a base-frame SE(3) pose with joint limits and explicit status codes."""
    try:
        target = np.asarray(target, dtype=float)
        hint = None if seed_q is None else np.asarray(seed_q, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return IKResult("invalid", "目标或初值包含无法解析的数值。")

    if target.shape != (4, 4) or not np.isfinite(target).all():
        return IKResult("invalid", "目标必须是有限的 4×4 齐次变换矩阵。")
    rotation = target[:3, :3]
    if (not np.allclose(target[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            or np.abs(rotation).max() > 1 + 1e-8
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-8, rtol=0)):
        return IKResult("invalid", "目标末行必须为 [0,0,0,1]，旋转块必须属于 SO(3)。")
    if hint is not None and (hint.shape != (6,) or not np.isfinite(hint).all()):
        return IKResult("invalid", "初值必须包含 6 个有限关节角（rad）。")

    position = target[:3, 3]
    shoulder = np.array([0, 0, DH[0, 1]])
    shoulder_offset = DH[2, 1]
    arm_length = DH[1, 0] + np.hypot(DH[2, 0], DH[3, 1])
    max_reach = np.hypot(arm_length, shoulder_offset)
    from_shoulder = position - shoulder
    if (np.any(np.abs(from_shoulder) > max_reach)
            or np.linalg.norm(from_shoulder) > max_reach + 1e-10
            or np.hypot(*position[:2]) < shoulder_offset - 1e-10):
        return IKResult("unreachable", "目标位置超出 PUMA 560 的几何工作空间。")

    def residual(q):
        actual = forward(q)
        return np.r_[actual[:3, 3] - position,
                     ROTATION_WEIGHT_M * (actual[:3, :3] - rotation).ravel()]

    def residual_jacobian(q):
        frames = forward_frames(q)
        jacobian = geometric_jacobian(frames)
        # dR/dq_i has columns z_i × R[:, j] for each tool-frame axis j.
        axes = jacobian[3:].T
        columns = np.cross(axes[:, None, :], frames[-1, :3, :3].T[None, :, :])
        rotation_derivatives = columns.transpose(0, 2, 1).reshape(6, 9).T
        return np.vstack((jacobian[:3], ROTATION_WEIGHT_M * rotation_derivatives))

    starts = [] if hint is None else [np.clip(hint, LOWER, UPPER)]
    starts.extend(np.deg2rad([[0, 40, -65, 0, 45, 0], [0, 100, -135, 0, -45, 0]]))
    rng = np.random.default_rng(560)
    starts.extend(rng.uniform(LOWER, UPPER, size=(18, 6)))
    best = None
    best_cost = np.inf
    for attempt, start in enumerate(starts, start=1):
        solution = least_squares(residual, start, jac=residual_jacobian, bounds=(LOWER, UPPER),
                                 ftol=1e-12, xtol=1e-12, gtol=1e-14, max_nfev=110)
        frames = forward_frames(solution.x)
        linear_error, angular_error = pose_errors(frames[-1], target)
        errors = (linear_error, angular_error)
        if solution.cost < best_cost:
            best_cost, best = solution.cost, errors
        if linear_error < POSITION_TOL and angular_error < ORIENTATION_TOL:
            jacobian = geometric_jacobian(frames)
            jacobian[3:] *= ROTATION_WEIGHT_M
            spectrum = np.linalg.svd(jacobian, compute_uv=False)
            ratio = float(spectrum[-1] / spectrum[0])
            status = "singular" if ratio < SINGULAR_RATIO else "converged"
            message = ("达到目标，但当前逆解处于或接近奇异位形，部分运动方向不可控。"
                       if status == "singular" else "位置与姿态均达到容差。")
            return IKResult(status, message, solution.x.tolist(), *errors, ratio, attempt)
    return IKResult("not_converged", "在关节限位内未收敛，目标可能不可达或需要不同初值。",
                    position_error_m=best[0], orientation_error_rad=best[1], attempts=len(starts))


def quintic_trajectory(q_start, q_finish, time, duration):
    """Return q, dq/dt, d²q/dt² for a rest-to-rest quintic joint move."""
    start, end = np.asarray(q_start, dtype=float), np.asarray(q_finish, dtype=float)
    time = np.asarray(time, dtype=float)
    if not np.isfinite(duration) or duration <= 0 or not np.isfinite(time).all():
        raise ValueError("时长必须为正，采样时间必须有限")
    if np.any(time < 0) or np.any(time > duration):
        raise ValueError("采样时间必须落在 [0, duration] 内")
    s = time / duration
    blend = s**3 * (10 + s * (-15 + 6 * s))
    rate = 30 * s**2 * (1 - s)**2 / duration
    acceleration = 60 * s * (1 - s) * (1 - 2 * s) / duration**2
    delta = end - start
    return start + blend[..., None] * delta, rate[..., None] * delta, acceleration[..., None] * delta
