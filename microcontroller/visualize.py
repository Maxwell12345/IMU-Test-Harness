import argparse
import bisect
import json
import math
import sqlite3
from pathlib import Path

import numpy as np


def q_norm(q):
    n = np.linalg.norm(q)
    if n <= 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / n


def q_conj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])

def q_to_R(q):
    x, y, z, w = q_norm(q)
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
    ])


def q_delta_angle(q0, q1):
    dq = q_mul(q_conj(q_norm(q0)), q_norm(q1))
    return 2.0 * math.atan2(np.linalg.norm(dq[:3]), abs(dq[3]))


def dominant_axis(v_body, a_body_window):
    if np.linalg.norm(v_body) > 0.15:
        return int(np.argmax(np.abs(v_body)))

    if len(a_body_window) >= 10:
        W = np.array(a_body_window)
        p = np.var(W, axis=0)
        return int(np.argmax(p))

    return 0


def q_rotate(q, v):
    p = np.array([v[0], v[1], v[2], 0.0])
    return q_mul(q_mul(q, p), q_conj(q))[:3]


def nearest_rotation(rot_t, rot_rows, t, max_dt):
    j = bisect.bisect_left(rot_t, t)
    best = None
    best_dt = None
    for idx in (j - 1, j):
        if 0 <= idx < len(rot_rows):
            dt = abs(rot_t[idx] - t)
            if best is None or dt < best_dt:
                best = rot_rows[idx]
                best_dt = dt
    if best is None or best_dt > max_dt:
        return None
    return best


def robust_update(x, P, z, H, R, chi2_gate):
    y = z - H @ x
    S = H @ P @ H.T + R
    try:
        Sinv = np.linalg.inv(S)
    except np.linalg.LinAlgError:
        Sinv = np.linalg.pinv(S)
    nis = float(y.T @ Sinv @ y)
    if nis > chi2_gate:
        scale = min(100.0, nis / chi2_gate)
        R = R * scale
        S = H @ P @ H.T + R
        try:
            Sinv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            Sinv = np.linalg.pinv(S)
    K = P @ H.T @ Sinv
    x = x + K @ y
    I = np.eye(P.shape[0])
    P = (I - K @ H) @ P @ (I - K @ H).T + K @ R @ K.T
    P = 0.5 * (P + P.T)
    innovation_cov = np.outer(y, y) - H @ P @ H.T
    return x, P, nis, innovation_cov


def load_data(db_path, time_source, max_sync_ms, invert_quat):
    db = sqlite3.connect(db_path)
    acc = db.execute(
        f"SELECT {time_source}, host_ns, dev_us, x, y, z FROM linear_acc ORDER BY {time_source}"
    ).fetchall()
    rot = db.execute(
        f"SELECT {time_source}, host_ns, dev_us, i, j, k, real, accuracy FROM rotation_vec ORDER BY {time_source}"
    ).fetchall()
    db.close()

    if not acc:
        raise RuntimeError("linear_acc table is empty")
    if not rot:
        raise RuntimeError("rotation_vec table is empty")

    rot_t = [float(r[0]) for r in rot]
    scale = 1e-6 if time_source == "dev_us" else 1e-9
    max_dt = max_sync_ms * (1000.0 if time_source == "dev_us" else 1_000_000.0)

    out = []
    for row in acc:
        t_raw, host_ns, dev_us, ax, ay, az = row
        rr = nearest_rotation(rot_t, rot, float(t_raw), max_dt)
        if rr is None:
            continue

        _, r_host_ns, r_dev_us, qi, qj, qk, qr, qacc = rr
        q = q_norm(np.array([qi, qj, qk, qr], dtype=float))
        if invert_quat:
            q = q_conj(q)

        a_body = np.array([ax, ay, az], dtype=float)
        a_world = q_rotate(q, a_body)
        t = float(t_raw) * scale

        if np.all(np.isfinite(a_world)) and np.all(np.isfinite(q)):
            out.append((t, a_world, q, float(qacc), int(host_ns), int(dev_us)))

    if len(out) < 5:
        raise RuntimeError("not enough synchronized IMU samples")

    return out


def run_filter(samples, no_smoother=True):
    n = len(samples)

    x = np.zeros(9)
    P = np.diag([0.05, 0.05, 0.05, 0.50, 0.50, 0.50, 0.20, 0.20, 0.20]) ** 2

    xs = []
    Ps = []
    xpreds = []
    Ppreds = []
    Fs = []

    stationary = []
    nis_values = []

    I3 = np.eye(3)
    Rv = np.eye(3) * 0.03 ** 2
    Rb = np.eye(3) * 0.08 ** 2
    Rnhc = np.eye(2) * 0.20 ** 2

    acc_window = []
    body_acc_window = []

    t0 = samples[0][0]
    last_t = samples[0][0]
    last_a = samples[0][1].copy()
    last_q = samples[0][2].copy()

    for idx, (t, a_world, q, qacc, host_ns, dev_us) in enumerate(samples):
        if idx == 0:
            dt = 0.005
        else:
            dt = t - last_t

        if not np.isfinite(dt) or dt <= 0.0:
            dt = 0.005

        dt = max(0.0005, min(0.05, dt))

        Rbw = q_to_R(q)
        Rwb = Rbw.T
        a_body = Rwb @ a_world

        acc_window.append(a_world.copy())
        body_acc_window.append(a_body.copy())

        if len(acc_window) > 120:
            acc_window.pop(0)
        if len(body_acc_window) > 120:
            body_acc_window.pop(0)

        W = np.array(acc_window)
        mean_a = W.mean(axis=0)
        std_a = float(np.sqrt(np.trace(np.cov(W.T))) if len(W) > 4 else 0.0)
        norm_a = float(np.linalg.norm(mean_a))
        jerk = float(np.linalg.norm(a_world - last_a) / dt)

        dq_angle = q_delta_angle(last_q, q)
        omega = min(dq_angle / dt, 20.0)

        speed = float(np.linalg.norm(x[3:6]))
        orient_sigma = max(0.0, min(abs(qacc), 0.75)) if np.isfinite(qacc) else 0.20
        orient_acc_noise = max(0.0, np.linalg.norm(a_world) * math.sin(orient_sigma))

        stationary_conf = math.exp(-((norm_a / 0.10) ** 2)) * math.exp(-((std_a / 0.065) ** 2)) * math.exp(-((jerk / 2.0) ** 2))
        is_stationary = stationary_conf > 0.55 and len(acc_window) >= 25

        sigma_a = 0.35 + 1.75 * std_a + 0.035 * min(jerk, 50.0) + 0.65 * orient_acc_noise + 0.65 * speed * omega
        sigma_b = 0.004 + 0.040 * (1.0 - min(stationary_conf, 1.0)) + 0.015 * min(std_a, 2.0)

        sigma_a = max(0.20, min(sigma_a, 25.0))
        sigma_b = max(0.001, min(sigma_b, 0.50))

        F = np.eye(9)
        F[0:3, 3:6] = dt * I3
        F[0:3, 6:9] = -0.5 * dt * dt * I3
        F[3:6, 6:9] = -dt * I3

        a_corr = a_world - x[6:9]

        x_pred = x.copy()
        x_pred[0:3] = x[0:3] + x[3:6] * dt + 0.5 * a_corr * dt * dt
        x_pred[3:6] = x[3:6] + a_corr * dt
        x_pred[6:9] = x[6:9]

        qa = sigma_a ** 2
        qb = sigma_b ** 2

        Q = np.zeros((9, 9))
        Q[0:3, 0:3] = I3 * qa * dt ** 4 / 4.0
        Q[0:3, 3:6] = I3 * qa * dt ** 3 / 2.0
        Q[3:6, 0:3] = I3 * qa * dt ** 3 / 2.0
        Q[3:6, 3:6] = I3 * qa * dt ** 2
        Q[6:9, 6:9] = I3 * qb * dt

        Q *= 12.0

        P_pred = F @ P @ F.T + Q
        P_pred = 0.5 * (P_pred + P_pred.T)

        x = x_pred
        P = P_pred

        nis = 0.0

        v_body = Rwb @ x[3:6]
        ax = dominant_axis(v_body, body_acc_window)
        perp = [i for i in range(3) if i != ax]

        if not is_stationary and speed > 0.05:
            H = np.zeros((2, 9))
            H[:, 3:6] = Rwb[perp, :]

            z = np.zeros(2)

            slip = np.linalg.norm(v_body[perp])
            sigma_nhc = 0.08 + 0.40 * min(std_a, 2.0) + 0.08 * speed + 0.12 * omega + 0.50 * min(slip, 3.0)
            sigma_nhc = max(0.04, min(sigma_nhc, 3.0))

            Rn = np.eye(2) * sigma_nhc ** 2
            Rn = 0.85 * Rnhc + 0.15 * Rn

            x, P, nis_n, icov = robust_update(x, P, z, H, Rn, 13.82)
            nis += nis_n

            diag = np.clip(np.diag(icov), 1e-6, 9.0)
            Rnhc = 0.97 * Rnhc + 0.03 * np.diag(diag)

        if is_stationary:
            H = np.zeros((3, 9))
            H[:, 3:6] = I3
            z = np.zeros(3)

            rv = (0.006 + 0.08 * (1.0 - stationary_conf) + 0.35 * std_a) ** 2
            Rzupt = np.eye(3) * max(rv, 1e-8)
            Rzupt = 0.85 * Rv + 0.15 * Rzupt

            x, P, nis_v, icov = robust_update(x, P, z, H, Rzupt, 16.27)
            nis += nis_v

            diag = np.clip(np.diag(icov), 1e-8, 0.50)
            Rv = 0.97 * Rv + 0.03 * np.diag(diag)

            H = np.zeros((3, 9))
            H[:, 6:9] = I3
            z = a_world.copy()

            rb = (0.015 + 0.12 * (1.0 - stationary_conf) + 0.80 * std_a + 0.30 * orient_acc_noise) ** 2
            Rbias = np.eye(3) * max(rb, 1e-8)
            Rbias = 0.90 * Rb + 0.10 * Rbias

            x, P, nis_b, icov = robust_update(x, P, z, H, Rbias, 16.27)
            nis += nis_b

            diag = np.clip(np.diag(icov), 1e-8, 2.0)
            Rb = 0.97 * Rb + 0.03 * np.diag(diag)

        xpreds.append(x_pred.copy())
        Ppreds.append(P_pred.copy())
        Fs.append(F.copy())
        xs.append(x.copy())
        Ps.append(P.copy())
        stationary.append(bool(is_stationary))
        nis_values.append(float(nis))

        last_t = t
        last_a = a_world.copy()
        last_q = q.copy()

    xs = np.array(xs)
    Ps = np.array(Ps)
    xpreds = np.array(xpreds)
    Ppreds = np.array(Ppreds)
    Fs = np.array(Fs)

    if not no_smoother and n > 3:
        x_s = xs.copy()
        P_s = Ps.copy()

        for k in range(n - 2, -1, -1):
            try:
                invp = np.linalg.inv(Ppreds[k + 1])
            except np.linalg.LinAlgError:
                invp = np.linalg.pinv(Ppreds[k + 1])

            C = Ps[k] @ Fs[k + 1].T @ invp
            x_s[k] = xs[k] + C @ (x_s[k + 1] - xpreds[k + 1])
            P_s[k] = Ps[k] + C @ (P_s[k + 1] - Ppreds[k + 1]) @ C.T
            P_s[k] = 0.5 * (P_s[k] + P_s[k].T)

        path = x_s[:, 0:3]
        vel = x_s[:, 3:6]
        bias = x_s[:, 6:9]
    else:
        path = xs[:, 0:3]
        vel = xs[:, 3:6]
        bias = xs[:, 6:9]

    times = np.array([s[0] - t0 for s in samples])
    qs = np.array([s[2] for s in samples])
    acc_world = np.array([s[1] for s in samples])
    qacc = np.array([s[3] for s in samples])

    return {
        "times": times,
        "path": path,
        "vel": vel,
        "bias": bias,
        "q": qs,
        "acc_world": acc_world,
        "qacc": qacc,
        "stationary": np.array(stationary),
        "nis": np.array(nis_values),
    }

def downsample_indices(n, max_points):
    if n <= max_points:
        return np.arange(n)
    idx = np.linspace(0, n - 1, max_points).astype(int)
    idx[0] = 0
    idx[-1] = n - 1
    return np.unique(idx)


def clean_path(path):
    good = np.all(np.isfinite(path), axis=1)
    if not np.any(good):
        raise RuntimeError("filter produced no finite positions")
    path = path.copy()
    first = path[good][0]
    for i in range(len(path)):
        if not good[i]:
            path[i] = first if i == 0 else path[i - 1]
    return path - path[0]


def html_escape_json(obj):
    return json.dumps(obj, separators=(",", ":")).replace("</", "<\\/")


def write_html(out_path, result, max_points):
    path = clean_path(result["path"])
    idx = downsample_indices(len(path), max_points)

    p = path[idx]
    q = result["q"][idx]
    times = result["times"][idx]
    stationary = result["stationary"][idx].astype(int)

    display = np.column_stack([p[:, 0], p[:, 2], p[:, 1]])

    stats = {
        "samples": int(len(path)),
        "rendered": int(len(idx)),
        "duration_s": float(result["times"][-1] if len(result["times"]) else 0.0),
        "distance_start_end_m": float(np.linalg.norm(path[-1] - path[0])),
        "max_radius_m": float(np.max(np.linalg.norm(path - path[0], axis=1))),
        "max_speed_mps": float(np.max(np.linalg.norm(result["vel"], axis=1))),
        "stationary_samples": int(np.sum(result["stationary"])),
    }

    data = {
        "points": display.round(6).tolist(),
        "q": q.round(8).tolist(),
        "t": times.round(4).tolist(),
        "stationary": stationary.tolist(),
        "stats": stats,
    }

    html = f"""<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>IMU Adaptive Dead Reckoning</title>
<style>
html,body{{margin:0;width:100%;height:100%;overflow:hidden;background:#050505;color:#eaeaea;font-family:Arial,sans-serif}}
#ui{{position:absolute;left:12px;top:12px;background:rgba(0,0,0,.72);padding:12px;border:1px solid #444;border-radius:8px;z-index:5;max-width:420px;font-size:13px;line-height:1.35}}
#bar{{position:absolute;left:12px;right:12px;bottom:12px;background:rgba(0,0,0,.72);padding:10px;border:1px solid #444;border-radius:8px;z-index:5;display:flex;gap:10px;align-items:center}}
#time{{width:100%}}
button{{background:#222;color:#eee;border:1px solid #666;border-radius:5px;padding:6px 10px}}
canvas{{display:block}}
</style>
</head>
<body>
<div id="ui">
<div><b>Adaptive ESKF Dead Reckoning</b></div>
<div>Click scene for mouse-look. ESC unlocks.</div>
<div>W/A/S/D move, Space up, Shift/Ctrl down, mouse look.</div>
<br>
<div id="stats"></div>
</div>
<div id="bar">
<button id="play">Play</button>
<input id="time" type="range" min="0" max="0" value="0">
<div id="readout">0</div>
</div>
<script src="https://unpkg.com/three@0.160.0/build/three.min.js"></script>
<script>
const DATA={html_escape_json(data)};
const pts=DATA.points.map(p=>new THREE.Vector3(p[0],p[1],p[2]));
const qs=DATA.q;
const scene=new THREE.Scene();
scene.background=new THREE.Color(0x050505);
const camera=new THREE.PerspectiveCamera(70,innerWidth/innerHeight,0.001,100000);
camera.rotation.order="YXZ";
const renderer=new THREE.WebGLRenderer({{antialias:true}});
renderer.setSize(innerWidth,innerHeight);
renderer.setPixelRatio(Math.min(devicePixelRatio,2));
document.body.appendChild(renderer.domElement);
const light1=new THREE.DirectionalLight(0xffffff,1.0);
light1.position.set(5,8,4);
scene.add(light1);
scene.add(new THREE.AmbientLight(0xffffff,.45));
const box=new THREE.Box3().setFromPoints(pts);
const center=new THREE.Vector3();
const size=new THREE.Vector3();
box.getCenter(center);
box.getSize(size);
const radius=Math.max(size.x,size.y,size.z,1);
const grid=new THREE.GridHelper(radius*3,40,0x444444,0x222222);
grid.position.y=box.min.y;
scene.add(grid);
scene.add(new THREE.AxesHelper(radius*.35));
const geom=new THREE.BufferGeometry().setFromPoints(pts);
const line=new THREE.Line(geom,new THREE.LineBasicMaterial({{color:0x55aaff}}));
scene.add(line);
function sphere(pos,r,color){{
    const m=new THREE.Mesh(new THREE.SphereGeometry(r,24,16),new THREE.MeshStandardMaterial({{color}}));
    m.position.copy(pos);
    scene.add(m);
    return m;
}}
sphere(pts[0],radius*.025,0x00ff66);
sphere(pts[pts.length-1],radius*.025,0xff3333);
const cubeSize=Math.max(radius*.035,.03);
const cube=new THREE.Mesh(new THREE.BoxGeometry(cubeSize,cubeSize*.45,cubeSize*.8),new THREE.MeshStandardMaterial({{color:0xffcc44,metalness:.1,roughness:.4}}));
scene.add(cube);
const trailGeom=new THREE.BufferGeometry();
const trailMat=new THREE.LineBasicMaterial({{color:0xffffff}});
const trail=new THREE.Line(trailGeom,trailMat);
scene.add(trail);
camera.position.set(center.x+radius*1.2,center.y+radius*.7,center.z+radius*1.2);
camera.lookAt(center);
let yaw=camera.rotation.y;
let pitch=camera.rotation.x;
let keys={{}};
let playing=false;
let cursor=pts.length-1;
const slider=document.getElementById("time");
slider.max=String(pts.length-1);
slider.value=String(cursor);
const readout=document.getElementById("readout");
const play=document.getElementById("play");
play.onclick=()=>{{playing=!playing;play.textContent=playing?"Pause":"Play"}};
slider.oninput=()=>{{cursor=Number(slider.value);updateCube()}};
document.addEventListener("keydown",e=>{{keys[e.code]=true}});
document.addEventListener("keyup",e=>{{keys[e.code]=false}});
renderer.domElement.addEventListener("click",()=>renderer.domElement.requestPointerLock());
document.addEventListener("mousemove",e=>{{
    if(document.pointerLockElement!==renderer.domElement)return;
    yaw-=e.movementX*0.002;
    pitch-=e.movementY*0.002;
    pitch=Math.max(-Math.PI/2+0.01,Math.min(Math.PI/2-0.01,pitch));
    camera.rotation.set(pitch,yaw,0);
}});
function setQuatFromData(i){{
    const q=qs[i];
    const tq=new THREE.Quaternion(q[0],q[2],q[1],q[3]);
    cube.quaternion.copy(tq);
}}
function updateCube(){{
    cursor=Math.max(0,Math.min(pts.length-1,cursor));
    cube.position.copy(pts[cursor]);
    setQuatFromData(cursor);
    slider.value=String(cursor);
    readout.textContent=DATA.t[cursor].toFixed(2)+" s";
    trailGeom.setFromPoints(pts.slice(0,cursor+1));
}}
function moveCamera(dt){{
    const speed=(keys.ShiftLeft||keys.ShiftRight||keys.ControlLeft||keys.ControlRight)?radius*.9:radius*.35;
    const dir=new THREE.Vector3();
    camera.getWorldDirection(dir);
    dir.y=0;
    if(dir.lengthSq()>0)dir.normalize();
    const right=new THREE.Vector3().crossVectors(dir,camera.up).normalize().multiplyScalar(-1);
    const step=speed*dt;
    if(keys.KeyW)camera.position.addScaledVector(dir,step);
    if(keys.KeyS)camera.position.addScaledVector(dir,-step);
    if(keys.KeyA)camera.position.addScaledVector(right,-step);
    if(keys.KeyD)camera.position.addScaledVector(right,step);
    if(keys.Space)camera.position.y+=step;
    if(keys.KeyQ||keys.ControlLeft||keys.ControlRight||keys.ShiftLeft||keys.ShiftRight)camera.position.y-=step;
}}
let last=performance.now();
function animate(now){{
    const dt=Math.min(.05,(now-last)/1000);
    last=now;
    if(playing){{
        cursor++;
        if(cursor>=pts.length)cursor=0;
        updateCube();
    }}
    moveCamera(dt);
    renderer.render(scene,camera);
    requestAnimationFrame(animate);
}}
addEventListener("resize",()=>{{
    camera.aspect=innerWidth/innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(innerWidth,innerHeight);
}});
document.getElementById("stats").innerHTML =
"Samples: "+DATA.stats.samples+"<br>"+
"Rendered: "+DATA.stats.rendered+"<br>"+
"Duration: "+DATA.stats.duration_s.toFixed(2)+" s<br>"+
"Start→End: "+DATA.stats.distance_start_end_m.toFixed(3)+" m<br>"+
"Max radius: "+DATA.stats.max_radius_m.toFixed(3)+" m<br>"+
"Max speed: "+DATA.stats.max_speed_mps.toFixed(3)+" m/s<br>"+
"Stationary samples: "+DATA.stats.stationary_samples;
updateCube();
requestAnimationFrame(animate);
</script>
</body>
</html>"""
    Path(out_path).write_text(html, encoding="utf-8")
    return stats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("db")
    ap.add_argument("--out", default="imu_deadreckon_viewer.html")
    ap.add_argument("--time-source", choices=["dev_us", "host_ns"], default="dev_us")
    ap.add_argument("--max-sync-ms", type=float, default=50.0)
    ap.add_argument("--max-points", type=int, default=25000)
    ap.add_argument("--invert-quat", action="store_true")
    ap.add_argument("--no-smoother", action="store_true")
    args = ap.parse_args()

    samples = load_data(args.db, args.time_source, args.max_sync_ms, args.invert_quat)
    result = run_filter(samples, True)
    stats = write_html(args.out, result, args.max_points)

    print(f"Wrote {args.out}")
    print(f"samples={stats['samples']} rendered={stats['rendered']} duration_s={stats['duration_s']:.3f}")
    print(f"start_end_m={stats['distance_start_end_m']:.3f} max_radius_m={stats['max_radius_m']:.3f} max_speed_mps={stats['max_speed_mps']:.3f}")


if __name__ == "__main__":
    main()