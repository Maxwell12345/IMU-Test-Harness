import argparse
import csv
import json
import os
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import serial
from serial import SerialException


HTML = r"""<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>Live IMU Orientation</title>
<style>
html, body {
    margin: 0;
    width: 100%;
    height: 100%;
    overflow: hidden;
    background: white;
    font-family: Arial, sans-serif;
}
#c {
    width: 100vw;
    height: 100vh;
    display: block;
}
#panel {
    position: fixed;
    right: 16px;
    top: 16px;
    background: rgba(255,255,255,0.92);
    border: 1px solid #999;
    border-radius: 8px;
    padding: 12px;
    font-size: 14px;
}
button {
    margin: 4px;
    padding: 6px 10px;
}
label {
    display: block;
    margin-top: 8px;
}
</style>
</head>
<body>
<canvas id="c"></canvas>
<div id="panel">
    <button id="zero">Zero / Level</button>
    <button id="reset">Reset Motion</button>
    <label>
        <input id="motionEnabled" type="checkbox" checked>
        motion enabled
    </label>
    <label>
        motion scale
        <input id="motionScale" type="range" min="1" max="30" value="10">
    </label>
    <label>
        accel deadband
        <input id="deadband" type="range" min="0" max="100" value="8">
    </label>
</div>
<script>
const canvas = document.getElementById("c");
const ctx = canvas.getContext("2d");

const zeroButton = document.getElementById("zero");
const resetButton = document.getElementById("reset");
const motionEnabled = document.getElementById("motionEnabled");
const motionScaleSlider = document.getElementById("motionScale");
const deadbandSlider = document.getElementById("deadband");

let latest = {
    R: [[1,0,0],[0,1,0],[0,0,1]],
    imu_time_us: 0,
    accuracy: 0,
    la: null,
    rv_count: 0,
    la_count: 0,
    line: "",
    la_host_time: 0,
    rv_host_time: 0
};

let zeroMatrix = [[1,0,0],[0,1,0],[0,0,1]];
let isZeroed = false;

let motion = {
    pos: [0, 0, 0],
    vel: [0, 0, 0],
    path: [],
    lastLaCount: 0,
    lastLaHostTime: null
};

const base = [
    [-1.0, -0.6, -0.15],
    [ 1.0, -0.6, -0.15],
    [ 1.0,  0.6, -0.15],
    [-1.0,  0.6, -0.15],
    [-1.0, -0.6,  0.15],
    [ 1.0, -0.6,  0.15],
    [ 1.0,  0.6,  0.15],
    [-1.0,  0.6,  0.15]
];

const edges = [
    [0,1], [1,2], [2,3], [3,0],
    [4,5], [5,6], [6,7], [7,4],
    [0,4], [1,5], [2,6], [3,7]
];

function identity() {
    return [[1,0,0],[0,1,0],[0,0,1]];
}

function transpose(A) {
    return [
        [A[0][0], A[1][0], A[2][0]],
        [A[0][1], A[1][1], A[2][1]],
        [A[0][2], A[1][2], A[2][2]]
    ];
}

function matMul(A, B) {
    return [
        [
            A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0],
            A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1],
            A[0][0] * B[0][2] + A[0][1] * B[1][2] + A[0][2] * B[2][2]
        ],
        [
            A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0],
            A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1],
            A[1][0] * B[0][2] + A[1][1] * B[1][2] + A[1][2] * B[2][2]
        ],
        [
            A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0],
            A[2][0] * B[0][1] + A[2][1] * B[1][1] + A[2][2] * B[2][1],
            A[2][0] * B[0][2] + A[2][1] * B[1][2] + A[2][2] * B[2][2]
        ]
    ];
}

function matVec(A, v) {
    return [
        A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
        A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
        A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2]
    ];
}

function add(a, b) {
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function scaleVec(a, s) {
    return [a[0] * s, a[1] * s, a[2] * s];
}

function norm(a) {
    return Math.hypot(a[0], a[1], a[2]);
}

function resetMotion() {
    motion.pos = [0, 0, 0];
    motion.vel = [0, 0, 0];
    motion.path = [];
    motion.lastLaCount = latest.la_count;
    motion.lastLaHostTime = latest.la_host_time || null;
}

function getDisplayR() {
    return matMul(zeroMatrix, latest.R);
}

function resize() {
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.floor(window.innerWidth * dpr);
    canvas.height = Math.floor(window.innerHeight * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

function normalize(v) {
    const n = Math.hypot(v[0], v[1], v[2]);
    return [v[0] / n, v[1] / n, v[2] / n];
}

function cross(a, b) {
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    ];
}

function dot(a, b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function project(points) {
    const camera = [3.0, -5.0, 3.0];
    const target = [0.0, 0.0, 0.0];
    const up = [0.0, 0.0, 1.0];

    const forward = normalize([
        target[0] - camera[0],
        target[1] - camera[1],
        target[2] - camera[2]
    ]);

    const right = normalize(cross(forward, up));
    const trueUp = cross(right, forward);

    return points.map(p => {
        const rel = [
            p[0] - camera[0],
            p[1] - camera[1],
            p[2] - camera[2]
        ];

        const x = dot(rel, right);
        const y = dot(rel, trueUp);
        const z = dot(rel, forward);
        const s = 5.0 / Math.max(1.0, 5.0 + z);

        return [x * s, y * s];
    });
}

function screen(p) {
    const w = window.innerWidth;
    const h = window.innerHeight;
    const s = Math.min(w, h) * 0.30;
    return [w * 0.5 + p[0] * s, h * 0.57 - p[1] * s];
}

function line(a, b, color, width) {
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.beginPath();
    ctx.moveTo(a[0], a[1]);
    ctx.lineTo(b[0], b[1]);
    ctx.stroke();
}

function integrateMotion() {
    if (!motionEnabled.checked) {
        motion.lastLaCount = latest.la_count;
        motion.lastLaHostTime = latest.la_host_time || motion.lastLaHostTime;
        return;
    }

    if (latest.la === null) {
        return;
    }

    if (latest.la_count === motion.lastLaCount) {
        return;
    }

    const now = latest.la_host_time;

    if (!now) {
        motion.lastLaCount = latest.la_count;
        return;
    }

    if (motion.lastLaHostTime === null) {
        motion.lastLaHostTime = now;
        motion.lastLaCount = latest.la_count;
        return;
    }

    let dt = now - motion.lastLaHostTime;

    if (dt <= 0 || dt > 0.25) {
        dt = 0.02;
    }

    const R = getDisplayR();
    let a = matVec(R, latest.la);
    const deadband = Number(deadbandSlider.value) / 100.0;

    if (norm(a) < deadband) {
        a = [0, 0, 0];
    }

    motion.vel[0] += a[0] * dt;
    motion.vel[1] += a[1] * dt;
    motion.vel[2] += a[2] * dt;

    const vDamp = Math.exp(-2.4 * dt);
    motion.vel[0] *= vDamp;
    motion.vel[1] *= vDamp;
    motion.vel[2] *= vDamp;

    motion.pos[0] += motion.vel[0] * dt;
    motion.pos[1] += motion.vel[1] * dt;
    motion.pos[2] += motion.vel[2] * dt;

    const pDamp = Math.exp(-0.35 * dt);
    motion.pos[0] *= pDamp;
    motion.pos[1] *= pDamp;
    motion.pos[2] *= pDamp;

    const pNorm = norm(motion.pos);

    if (pNorm > 1.5) {
        motion.pos = scaleVec(motion.pos, 1.5 / pNorm);
        motion.vel = scaleVec(motion.vel, 0.25);
    }

    motion.path.push([...motion.pos]);

    if (motion.path.length > 500) {
        motion.path.shift();
    }

    motion.lastLaHostTime = now;
    motion.lastLaCount = latest.la_count;
}

function drawPath(scale) {
    if (motion.path.length < 2) {
        return;
    }

    const pts3 = motion.path.map(p => scaleVec(p, scale));
    const pts2 = project(pts3).map(screen);

    ctx.strokeStyle = "#777";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(pts2[0][0], pts2[0][1]);

    for (let i = 1; i < pts2.length; i++) {
        ctx.lineTo(pts2[i][0], pts2[i][1]);
    }

    ctx.stroke();
}

function draw() {
    integrateMotion();

    ctx.clearRect(0, 0, window.innerWidth, window.innerHeight);

    const R = getDisplayR();
    const motionScale = Number(motionScaleSlider.value);
    const center = scaleVec(motion.pos, motionScale);

    drawPath(motionScale);

    const verts = base.map(p => add(matVec(R, p), center));
    const projected = project(verts).map(screen);

    for (const e of edges) {
        line(projected[e[0]], projected[e[1]], "black", 3);
    }

    const axes3 = [
        center,
        add(center, matVec(R, [1.5, 0.0, 0.0])),
        add(center, matVec(R, [0.0, 1.5, 0.0])),
        add(center, matVec(R, [0.0, 0.0, 1.5]))
    ];

    const axes2 = project(axes3).map(screen);

    line(axes2[0], axes2[1], "red", 4);
    line(axes2[0], axes2[2], "green", 4);
    line(axes2[0], axes2[3], "blue", 4);

    ctx.font = "bold 18px Arial";
    ctx.fillStyle = "red";
    ctx.fillText("X", axes2[1][0], axes2[1][1]);
    ctx.fillStyle = "green";
    ctx.fillText("Y", axes2[2][0], axes2[2][1]);
    ctx.fillStyle = "blue";
    ctx.fillText("Z", axes2[3][0], axes2[3][1]);

    let laText = "LA: none";

    if (latest.la !== null) {
        laText = `LA: ${latest.la[0].toFixed(3)}, ${latest.la[1].toFixed(3)}, ${latest.la[2].toFixed(3)}`;
    }

    const text = [
        `RV count: ${latest.rv_count}`,
        `LA count: ${latest.la_count}`,
        `imu_time_us: ${latest.imu_time_us}`,
        `accuracy: ${Number(latest.accuracy).toFixed(3)}`,
        `zeroed: ${isZeroed ? "yes" : "no"}`,
        `motion pos: ${motion.pos[0].toFixed(3)}, ${motion.pos[1].toFixed(3)}, ${motion.pos[2].toFixed(3)}`,
        `motion vel: ${motion.vel[0].toFixed(3)}, ${motion.vel[1].toFixed(3)}, ${motion.vel[2].toFixed(3)}`,
        laText,
        latest.line
    ];

    ctx.fillStyle = "black";
    ctx.font = "16px Arial";
    let y = 24;

    for (const t of text) {
        ctx.fillText(t, 16, y);
        y += 22;
    }

    requestAnimationFrame(draw);
}

async function poll() {
    try {
        const r = await fetch("/state", { cache: "no-store" });
        latest = await r.json();
    } catch (e) {
    }

    setTimeout(poll, 25);
}

zeroButton.onclick = () => {
    zeroMatrix = transpose(latest.R);
    isZeroed = true;
    resetMotion();
};

resetButton.onclick = () => {
    resetMotion();
};

window.addEventListener("keydown", e => {
    if (e.key === " " || e.key.toLowerCase() === "z") {
        zeroMatrix = transpose(latest.R);
        isZeroed = true;
        resetMotion();
    }

    if (e.key.toLowerCase() === "r") {
        resetMotion();
    }
});

window.addEventListener("resize", resize);
resize();
poll();
draw();
</script>
</body>
</html>
"""


def open_port(port, baud):
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 1
    ser.write_timeout = 1
    ser.rtscts = False
    ser.dsrdtr = False

    if os.name == "posix":
        ser.exclusive = True

    ser.open()
    ser.dtr = False
    ser.rts = False
    time.sleep(0.25)
    return ser


def parse_line(line):
    parts = line.split(",")

    try:
        if len(parts) == 5 and parts[0] == "LA":
            return {
                "type": "LA",
                "host_time": time.time(),
                "imu_time_us": int(parts[1]),
                "x": float(parts[2]),
                "y": float(parts[3]),
                "z": float(parts[4]),
                "i": "",
                "j": "",
                "k": "",
                "real": "",
                "accuracy": "",
            }

        if len(parts) == 7 and parts[0] == "RV":
            return {
                "type": "RV",
                "host_time": time.time(),
                "imu_time_us": int(parts[1]),
                "x": "",
                "y": "",
                "z": "",
                "i": float(parts[2]),
                "j": float(parts[3]),
                "k": float(parts[4]),
                "real": float(parts[5]),
                "accuracy": float(parts[6]),
            }
    except ValueError:
        return None

    return None


def quat_to_matrix(i, j, k, real):
    q = np.array([real, i, j, k], dtype=float)
    n = np.linalg.norm(q)

    if n <= 0.0:
        return np.eye(3)

    w, x, y, z = q / n

    return np.array([
        [1.0 - 2.0 * y * y - 2.0 * z * z, 2.0 * x * y - 2.0 * z * w, 2.0 * x * z + 2.0 * y * w],
        [2.0 * x * y + 2.0 * z * w, 1.0 - 2.0 * x * x - 2.0 * z * z, 2.0 * y * z - 2.0 * x * w],
        [2.0 * x * z - 2.0 * y * w, 2.0 * y * z + 2.0 * x * w, 1.0 - 2.0 * x * x - 2.0 * y * y],
    ])


def serial_worker(args, state, lock, stop_event):
    fieldnames = [
        "type",
        "host_time",
        "imu_time_us",
        "x",
        "y",
        "z",
        "i",
        "j",
        "k",
        "real",
        "accuracy",
    ]

    ser = None

    with open(args.output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        while not stop_event.is_set():
            try:
                ser = open_port(args.port, args.baud)
                print(f"connected,{args.port},{args.baud}")

                while not stop_event.is_set():
                    raw = ser.readline()

                    if not raw:
                        continue

                    line = raw.decode("utf-8", errors="ignore").strip()

                    if not line:
                        continue

                    if args.print_lines:
                        print(line)

                    row = parse_line(line)

                    if row is None:
                        continue

                    writer.writerow(row)
                    f.flush()

                    with lock:
                        state["line"] = line

                        if row["type"] == "RV":
                            state["R"] = quat_to_matrix(row["i"], row["j"], row["k"], row["real"]).tolist()
                            state["imu_time_us"] = row["imu_time_us"]
                            state["accuracy"] = row["accuracy"]
                            state["rv_count"] += 1
                            state["rv_host_time"] = row["host_time"]

                        if row["type"] == "LA":
                            state["la"] = [row["x"], row["y"], row["z"]]
                            state["la_count"] += 1
                            state["la_host_time"] = row["host_time"]

            except SerialException as e:
                print(f"serial_error,{e}")

                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass

                ser = None
                time.sleep(1)

            except Exception as e:
                print(f"error,{e}")

                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass

                ser = None
                time.sleep(1)

        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass


def make_handler(state, lock):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def do_GET(self):
            if self.path == "/" or self.path.startswith("/?"):
                data = HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return

            if self.path == "/state":
                with lock:
                    snapshot = dict(state)

                data = json.dumps(snapshot).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return

            self.send_response(404)
            self.end_headers()

    return Handler


def open_browser(url):
    try:
        subprocess.Popen(
            ["cmd.exe", "/c", "start", "", url],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("-b", "--baud", type=int, default=115200)
    parser.add_argument("-o", "--output", default="imu_log.csv")
    parser.add_argument("--print-lines", action="store_true")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--web-port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()

    lock = threading.Lock()
    stop_event = threading.Event()

    state = {
        "R": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        "imu_time_us": 0,
        "accuracy": 0.0,
        "la": None,
        "rv_count": 0,
        "la_count": 0,
        "line": "",
        "la_host_time": 0.0,
        "rv_host_time": 0.0,
    }

    thread = threading.Thread(
        target=serial_worker,
        args=(args, state, lock, stop_event),
        daemon=True,
    )
    thread.start()

    server = ThreadingHTTPServer((args.host, args.web_port), make_handler(state, lock))
    url = f"http://localhost:{args.web_port}"

    print(f"viewer,{url}")

    if not args.no_browser:
        open_browser(url)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        server.shutdown()
        server.server_close()
        print("stopped")


if __name__ == "__main__":
    main()