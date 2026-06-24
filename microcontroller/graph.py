import argparse
import math
import sqlite3

import matplotlib.pyplot as plt
import numpy as np


AXES = {
    "x": np.array([1.0, 0.0, 0.0]),
    "y": np.array([0.0, 1.0, 0.0]),
    "z": np.array([0.0, 0.0, 1.0]),
    "-x": np.array([-1.0, 0.0, 0.0]),
    "-y": np.array([0.0, -1.0, 0.0]),
    "-z": np.array([0.0, 0.0, -1.0]),
}


def load(db_path):
    db = sqlite3.connect(db_path)

    la = np.array(db.execute("""
        SELECT dev_us, x, y, z
        FROM linear_acc
        ORDER BY dev_us
    """).fetchall(), dtype=float)

    rv = np.array(db.execute("""
        SELECT dev_us, i, j, k, real, accuracy
        FROM rotation_vec
        WHERE real BETWEEN -1.0 AND 1.0
          AND i BETWEEN -1.0 AND 1.0
          AND j BETWEEN -1.0 AND 1.0
          AND k BETWEEN -1.0 AND 1.0
          AND (i*i + j*j + k*k + real*real) BETWEEN 0.8 AND 1.2
        ORDER BY dev_us
    """).fetchall(), dtype=float)

    db.close()
    return la, rv


def normalize_quats(q):
    q = q.copy()
    q /= np.linalg.norm(q, axis=1)[:, None]

    for n in range(1, len(q)):
        if np.dot(q[n - 1], q[n]) < 0.0:
            q[n] = -q[n]

    return q


def interp_quats(la_t, rv_t, q):
    good = (la_t >= rv_t[0]) & (la_t <= rv_t[-1])
    out = np.zeros((np.count_nonzero(good), 4), dtype=float)

    for c in range(4):
        out[:, c] = np.interp(la_t[good], rv_t, q[:, c])

    out /= np.linalg.norm(out, axis=1)[:, None]
    return good, out


def quat_to_matrix(q):
    w = q[:, 0]
    x = q[:, 1]
    y = q[:, 2]
    z = q[:, 3]

    r = np.zeros((len(q), 3, 3), dtype=float)

    r[:, 0, 0] = 1.0 - 2.0 * (y * y + z * z)
    r[:, 0, 1] = 2.0 * (x * y - w * z)
    r[:, 0, 2] = 2.0 * (x * z + w * y)

    r[:, 1, 0] = 2.0 * (x * y + w * z)
    r[:, 1, 1] = 1.0 - 2.0 * (x * x + z * z)
    r[:, 1, 2] = 2.0 * (y * z - w * x)

    r[:, 2, 0] = 2.0 * (x * z - w * y)
    r[:, 2, 1] = 2.0 * (y * z + w * x)
    r[:, 2, 2] = 1.0 - 2.0 * (x * x + y * y)

    return r


def moving_mean(x, n):
    n = max(1, int(n))

    if n % 2 == 0:
        n += 1

    if n == 1:
        return x.copy()

    p = n // 2
    xp = np.pad(x, (p, p), mode="edge")
    return np.convolve(xp, np.ones(n) / n, mode="valid")


def rotate_declination(v, deg):
    d = math.radians(deg)
    c = math.cos(d)
    s = math.sin(d)

    out = v.copy()
    x = v[:, 0].copy()
    y = v[:, 1].copy()

    out[:, 0] = c * x + s * y
    out[:, 1] = -s * x + c * y

    return out


def build_candidate(t, a_sensor, q, direction, axis, declination, smooth_seconds, min_yaw_rate, max_speed):
    r = quat_to_matrix(q)

    if direction == "sensor_to_world":
        a_world = np.einsum("nij,nj->ni", r, a_sensor)
        fwd = np.einsum("nij,j->ni", r, AXES[axis])
    else:
        a_world = np.einsum("nji,nj->ni", r, a_sensor)
        fwd = np.einsum("nji,j->ni", r, AXES[axis])

    a_world = rotate_declination(a_world, declination)
    fwd = rotate_declination(fwd, declination)

    h = np.linalg.norm(fwd[:, :2], axis=1)
    h_safe = np.maximum(h, 1e-9)

    f2 = fwd[:, :2] / h_safe[:, None]
    left = np.column_stack((-f2[:, 1], f2[:, 0]))

    heading = np.unwrap(np.arctan2(f2[:, 0], f2[:, 1]))
    yaw_rate = np.gradient(heading, t)

    dt_med = np.median(np.diff(t))
    win = max(3, int(round(smooth_seconds / dt_med)))

    yaw_rate_s = moving_mean(yaw_rate, win)
    a_fwd = a_world[:, 0] * f2[:, 0] + a_world[:, 1] * f2[:, 1]
    a_left = a_world[:, 0] * left[:, 0] + a_world[:, 1] * left[:, 1]

    a_fwd_s = moving_mean(a_fwd, win)
    a_left_s = moving_mean(a_left, win)

    valid = np.abs(yaw_rate_s) >= min_yaw_rate
    v_turn = np.full(len(t), np.nan)

    v_turn[valid] = np.abs(a_left_s[valid] / yaw_rate_s[valid])
    valid &= np.isfinite(v_turn)
    valid &= v_turn >= 0.0
    valid &= v_turn <= max_speed
    valid &= h > 0.2

    if np.count_nonzero(valid) < 5:
        speed = np.zeros(len(t))
    else:
        speed = np.interp(t, t[valid], v_turn[valid])
        speed = moving_mean(speed, win)
        speed = np.clip(speed, 0.0, max_speed)

    pos = np.zeros((len(t), 2))

    for n in range(1, len(t)):
        dt = t[n] - t[n - 1]
        pos[n] = pos[n - 1] + speed[n] * f2[n] * dt

    dist = np.linalg.norm(pos[-1] - pos[0])
    path_len = np.sum(np.linalg.norm(np.diff(pos, axis=0), axis=1))
    closure = dist / path_len if path_len > 1e-9 else 999.0
    heading_change = math.degrees(heading[-1] - heading[0])
    valid_frac = float(np.mean(valid))
    horiz = float(np.median(h))
    med_speed = float(np.median(speed))
    score = abs(heading_change) * valid_frac * horiz

    return {
        "direction": direction,
        "axis": axis,
        "score": score,
        "t": t,
        "pos": pos,
        "heading": heading,
        "yaw_rate": yaw_rate_s,
        "speed": speed,
        "a_fwd": a_fwd_s,
        "a_left": a_left_s,
        "heading_change": heading_change,
        "valid_frac": valid_frac,
        "median_speed": med_speed,
        "closure": closure,
        "a_world": a_world,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("db")
    parser.add_argument("--declination", type=float, default=0.0)
    parser.add_argument("--quat-direction", choices=["auto", "sensor_to_world", "world_to_sensor"], default="auto")
    parser.add_argument("--forward-axis", choices=["auto", "x", "y", "z", "-x", "-y", "-z"], default="auto")
    parser.add_argument("--smooth-seconds", type=float, default=0.75)
    parser.add_argument("--min-yaw-rate", type=float, default=0.03)
    parser.add_argument("--max-speed", type=float, default=15.0)
    args = parser.parse_args()

    la, rv = load(args.db)

    if len(la) < 3:
        raise RuntimeError("not enough linear_acc rows")

    if len(rv) < 3:
        raise RuntimeError("not enough rotation_vec rows")

    la_t = la[:, 0]
    rv_t = rv[:, 0]

    q = np.column_stack((rv[:, 4], rv[:, 1], rv[:, 2], rv[:, 3]))
    q = normalize_quats(q)

    good, q_match = interp_quats(la_t, rv_t, q)

    la = la[good]
    la_t = la_t[good]

    if len(la) < 3:
        raise RuntimeError("not enough matched LA/RV rows")

    t = (la_t - la_t[0]) / 1e6
    a_sensor = la[:, 1:4]

    directions = ["sensor_to_world", "world_to_sensor"] if args.quat_direction == "auto" else [args.quat_direction]
    axes = list(AXES.keys()) if args.forward_axis == "auto" else [args.forward_axis]

    candidates = []

    for direction in directions:
        for axis in axes:
            try:
                candidates.append(build_candidate(
                    t,
                    a_sensor,
                    q_match,
                    direction,
                    axis,
                    args.declination,
                    args.smooth_seconds,
                    args.min_yaw_rate,
                    args.max_speed,
                ))
            except Exception:
                pass

    if len(candidates) == 0:
        raise RuntimeError("no usable candidates")

    candidates.sort(key=lambda x: x["score"], reverse=True)
    best = candidates[0]

    print("top candidates")
    print("score direction axis heading_change_deg valid_turn_frac median_speed_mps closure")
    for c in candidates[:12]:
        print(f"{c['score']:.3f} {c['direction']} {c['axis']} {c['heading_change']:.3f} {c['valid_frac']:.3f} {c['median_speed']:.3f} {c['closure']:.3f}")

    print()
    print(f"selected direction: {best['direction']}")
    print(f"selected forward axis: {best['axis']}")
    print(f"heading change degrees: {best['heading_change']:.3f}")
    print(f"median speed m/s: {best['median_speed']:.3f}")
    print(f"median speed mph: {best['median_speed'] * 2.236936:.3f}")
    print(f"final east meters: {best['pos'][-1, 0]:.3f}")
    print(f"final north meters: {best['pos'][-1, 1]:.3f}")

    plt.figure()
    plt.plot(best["pos"][:, 0], best["pos"][:, 1], label="turn-model path")
    plt.scatter(best["pos"][0, 0], best["pos"][0, 1], label="start")
    plt.scatter(best["pos"][-1, 0], best["pos"][-1, 1], label="end")
    plt.title("2D path from heading and turn acceleration")
    plt.xlabel("east meters")
    plt.ylabel("north meters")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.savefig("position_plot.png", dpi=300, bbox_inches="tight")
    plt.close()

    plt.figure()
    plt.plot(best["t"], np.degrees(best["heading"]), label="unwrapped heading")
    plt.title("heading from rotation vector")
    plt.xlabel("time seconds")
    plt.ylabel("degrees")
    plt.grid(True)
    plt.legend()
    plt.savefig("heading_plot.png", dpi=300, bbox_inches="tight")
    plt.close()

    plt.figure()
    plt.plot(best["t"], best["yaw_rate"], label="yaw rate")
    plt.title("yaw rate from rotation vector")
    plt.xlabel("time seconds")
    plt.ylabel("rad/s")
    plt.grid(True)
    plt.legend()
    plt.savefig("yaw_rate_plot.png", dpi=300, bbox_inches="tight")
    plt.close()

    plt.figure()
    plt.plot(best["t"], best["speed"], label="estimated speed")
    plt.title("speed estimated from lateral acceleration and yaw rate")
    plt.xlabel("time seconds")
    plt.ylabel("m/s")
    plt.grid(True)
    plt.legend()
    plt.savefig("speed_plot.png", dpi=300, bbox_inches="tight")
    plt.close()

    plt.figure()
    plt.plot(best["t"], best["a_fwd"], label="forward accel")
    plt.plot(best["t"], best["a_left"], label="left accel")
    plt.title("vehicle-frame acceleration")
    plt.xlabel("time seconds")
    plt.ylabel("m/s^2")
    plt.grid(True)
    plt.legend()
    plt.savefig("vehicle_accel_plot.png", dpi=300, bbox_inches="tight")
    plt.close()


if __name__ == "__main__":
    main()