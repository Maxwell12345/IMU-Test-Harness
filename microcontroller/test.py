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
<title>Live IMU Adaptive 3D Kinematics</title>
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
    width: 280px;
}
button {
    margin: 4px;
    padding: 6px 10px;
}
label {
    display: block;
    margin-top: 8px;
}
input[type="range"] {
    width: 100%;
}
.small {
    font-size: 12px;
    color: #333;
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
        <input id="adaptiveRest" type="checkbox" checked>
        adaptive rest lock
    </label>
    <label>
        <input id="adaptiveBias" type="checkbox" checked>
        adaptive bias estimate
    </label>
    <label>
        motion scale
        <input id="motionScale" type="range" min="1" max="80" value="12">
    </label>
    <label>
        drift damping
        <input id="damping" type="range" min="0" max="100" value="2">
    </label>
    <label>
        gate strength
        <input id="gateStrength" type="range" min="20" max="200" value="100">
    </label>
    <label>
        rest strictness
        <input id="restStrictness" type="range" min="1" max="100" value="35">
    </label>
    <div class="small">WASD move, Q/E down/up, click+drag look, Z zero, R reset</div>
</div>
<script>
const canvas = document.getElementById("c");
const ctx = canvas.getContext("2d");

const zeroButton = document.getElementById("zero");
const resetButton = document.getElementById("reset");
const motionEnabled = document.getElementById("motionEnabled");
const adaptiveRest = document.getElementById("adaptiveRest");
const adaptiveBias = document.getElementById("adaptiveBias");
const motionScaleSlider = document.getElementById("motionScale");
const dampingSlider = document.getElementById("damping");
const gateStrengthSlider = document.getElementById("gateStrength");
const restStrictnessSlider = document.getElementById("restStrictness");

let latest = {
    R: [[1,0,0],[0,1,0],[0,0,1]],
    imu_time_us: 0,
    accuracy: 0,
    la: null,
    rv_count: 0,
    la_count: 0,
    line: "",
    la_host_time: 0,
    rv_host_time: 0,
    la_imu_time_us: 0,
    la_samples: []
};

let zeroMatrix = [[1,0,0],[0,1,0],[0,0,1]];
let isZeroed = false;

let camera = {
    pos: [3.0, -5.0, 3.0],
    yaw: Math.atan2(5.0, -3.0),
    pitch: -0.45,
    speed: 6.0,
    mouseSensitivity: 0.0025,
    lastFrameTime: performance.now(),
    keys: {}
};

let motion = {
    pos: [0, 0, 0],
    vel: [0, 0, 0],
    bias: [0, 0, 0],
    noise: [0.06, 0.06, 0.06],
    pPos: [0, 0, 0],
    pVel: [0, 0, 0],
    path: [],
    lastLaCount: 0,
    lastLaImuTimeUs: null,
    lastAccelRaw: null,
    lastAccelUse: null,
    lastR: null,
    stillTime: 0,
    accepted: 0,
    rejected: 0,
    maha: 0,
    angularRate: 0,
    jerk: 0,
    confidence: 1,
    status: "init"
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

function sub(a, b) {
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function scaleVec(a, s) {
    return [a[0] * s, a[1] * s, a[2] * s];
}

function norm(a) {
    return Math.hypot(a[0], a[1], a[2]);
}

function dot(a, b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function cross(a, b) {
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    ];
}

function normalize(v) {
    const n = Math.max(1e-12, norm(v));
    return [v[0] / n, v[1] / n, v[2] / n];
}

function clamp(x, a, b) {
    return Math.max(a, Math.min(b, x));
}

function smoothAlpha(dt, tau) {
    return 1.0 - Math.exp(-dt / Math.max(1e-6, tau));
}

function rotationAngle(A, B) {
    const C = matMul(transpose(A), B);
    const tr = C[0][0] + C[1][1] + C[2][2];
    return Math.acos(clamp((tr - 1.0) * 0.5, -1.0, 1.0));
}

function getDisplayRFrom(R) {
    return matMul(zeroMatrix, R || latest.R);
}

function getDisplayR() {
    return getDisplayRFrom(latest.R);
}

function resetMotion() {
    motion.pos = [0, 0, 0];
    motion.vel = [0, 0, 0];
    motion.bias = [0, 0, 0];
    motion.noise = [0.06, 0.06, 0.06];
    motion.pPos = [0, 0, 0];
    motion.pVel = [0, 0, 0];
    motion.path = [];
    motion.lastLaCount = latest.la_count;
    motion.lastLaImuTimeUs = latest.la_imu_time_us || null;
    motion.lastAccelRaw = null;
    motion.lastAccelUse = null;
    motion.lastR = null;
    motion.stillTime = 0;
    motion.accepted = 0;
    motion.rejected = 0;
    motion.maha = 0;
    motion.angularRate = 0;
    motion.jerk = 0;
    motion.confidence = 1;
    motion.status = "reset";
}

function resize() {
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.floor(window.innerWidth * dpr);
    canvas.height = Math.floor(window.innerHeight * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

function getCameraBasis() {
    const cp = Math.cos(camera.pitch);
    const sp = Math.sin(camera.pitch);
    const cy = Math.cos(camera.yaw);
    const sy = Math.sin(camera.yaw);

    const forward = normalize([cp * cy, cp * sy, sp]);
    const worldUp = [0.0, 0.0, 1.0];
    const right = normalize(cross(forward, worldUp));
    const trueUp = cross(right, forward);

    return { forward, right, trueUp };
}

function updateCamera() {
    const now = performance.now();
    const dt = Math.min(0.05, (now - camera.lastFrameTime) / 1000.0);
    camera.lastFrameTime = now;

    const b = getCameraBasis();
    const flatForward = normalize([b.forward[0], b.forward[1], 0.0]);
    const flatRight = normalize([b.right[0], b.right[1], 0.0]);
    const step = camera.speed * dt;

    if (camera.keys["w"]) camera.pos = add(camera.pos, scaleVec(flatForward, step));
    if (camera.keys["s"]) camera.pos = add(camera.pos, scaleVec(flatForward, -step));
    if (camera.keys["d"]) camera.pos = add(camera.pos, scaleVec(flatRight, step));
    if (camera.keys["a"]) camera.pos = add(camera.pos, scaleVec(flatRight, -step));
    if (camera.keys["q"]) camera.pos[2] -= step;
    if (camera.keys["e"]) camera.pos[2] += step;
}

function project(points) {
    const b = getCameraBasis();

    return points.map(p => {
        const rel = [
            p[0] - camera.pos[0],
            p[1] - camera.pos[1],
            p[2] - camera.pos[2]
        ];

        const x = dot(rel, b.right);
        const y = dot(rel, b.trueUp);
        const z = dot(rel, b.forward);

        if (z <= 0.02) {
            return [1000000, 1000000];
        }

        const s = 2.5 / z;
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

function integrateLaSample(nowUs, la, rawR, rvAccuracy) {
    if (!nowUs) {
        return;
    }

    if (motion.lastLaImuTimeUs === null) {
        motion.lastLaImuTimeUs = nowUs;
        motion.lastR = getDisplayRFrom(rawR);
        return;
    }

    if (nowUs <= motion.lastLaImuTimeUs) {
        return;
    }

    let dt = (nowUs - motion.lastLaImuTimeUs) / 1000000.0;

    if (dt > 0.5) {
        motion.lastLaImuTimeUs = nowUs;
        motion.lastAccelRaw = null;
        motion.lastAccelUse = null;
        motion.lastR = getDisplayRFrom(rawR);
        motion.status = "gap reset";
        return;
    }

    dt = clamp(dt, 0.0005, 0.05);

    const R = getDisplayRFrom(rawR);
    const aRaw = matVec(R, la);
    const accelNorm = norm(la);
    const absAccelNorm = norm(aRaw);
    const angle = motion.lastR === null ? 0.0 : rotationAngle(motion.lastR, R);
    const angularRate = angle / dt;
    const accQuality = Number.isFinite(rvAccuracy) ? clamp(1.0 - rvAccuracy / 1.2, 0.15, 1.0) : 0.65;
    const restStrictness = Number(restStrictnessSlider.value) / 100.0;
    const restAccel = 0.025 + 0.22 * restStrictness;
    const restAngular = 0.015 + 0.30 * restStrictness;
    const restVel = 0.03 + 0.35 * restStrictness;
    const damping = Number(dampingSlider.value) / 1000.0;
    const gateScale = Number(gateStrengthSlider.value) / 100.0;
    let usedAccel = aRaw;
    let maha = 0.0;
    let rejected = false;

    if (motion.lastAccelRaw !== null) {
        const da = sub(aRaw, motion.lastAccelRaw);
        motion.jerk = norm(da) / dt;

        for (let i = 0; i < 3; i++) {
            const s = Math.max(0.015, motion.noise[i]);
            maha += (da[i] * da[i]) / (s * s + 1e-9);
        }

        const chi99 = 11.3449 * gateScale * gateScale;
        const impossible = absAccelNorm > 60.0 || motion.jerk > 250.0;
        const spike = maha > chi99 && absAccelNorm > Math.max(1.0, 3.0 * norm(motion.noise));

        if (impossible || spike) {
            rejected = true;
            usedAccel = motion.lastAccelRaw;
            motion.rejected += 1;
        } else {
            const beta = smoothAlpha(dt, 0.8);

            for (let i = 0; i < 3; i++) {
                const e = Math.abs(da[i]);
                motion.noise[i] = clamp((1.0 - beta) * motion.noise[i] + beta * e, 0.015, 6.0);
            }
        }
    }

    motion.maha = maha;
    motion.angularRate = angularRate;

    const speed = norm(motion.vel);
    const lowDynamics = accelNorm < restAccel && angularRate < restAngular;
    const velocityAllowsRest = speed < restVel;
    const restCandidate = adaptiveRest.checked && lowDynamics && velocityAllowsRest;

    if (restCandidate) {
        motion.stillTime += dt;
    } else {
        motion.stillTime = Math.max(0.0, motion.stillTime - 3.0 * dt);
    }

    const stationary = motion.stillTime > 0.45;

    if (adaptiveBias.checked) {
        const tau = stationary ? 0.25 : 90.0;
        const alpha = smoothAlpha(dt, tau);

        if (stationary || accelNorm < restAccel * 1.5) {
            for (let i = 0; i < 3; i++) {
                motion.bias[i] = (1.0 - alpha) * motion.bias[i] + alpha * usedAccel[i];
            }
        }
    }

    const a = sub(usedAccel, motion.bias);
    let aUse = a;

    if (motion.lastAccelUse !== null) {
        aUse = [
            0.5 * (motion.lastAccelUse[0] + a[0]),
            0.5 * (motion.lastAccelUse[1] + a[1]),
            0.5 * (motion.lastAccelUse[2] + a[2])
        ];
    }

    if (stationary) {
        const decay = Math.exp(-dt / 0.08);
        motion.vel[0] *= decay;
        motion.vel[1] *= decay;
        motion.vel[2] *= decay;

        if (norm(motion.vel) < 0.006) {
            motion.vel = [0, 0, 0];
        }

        motion.pVel = scaleVec(motion.pVel, Math.exp(-dt / 0.15));
        motion.status = "rest lock";
    } else {
        motion.vel[0] += aUse[0] * dt;
        motion.vel[1] += aUse[1] * dt;
        motion.vel[2] += aUse[2] * dt;

        if (damping > 0.0) {
            const d = Math.exp(-damping * dt);
            motion.vel[0] *= d;
            motion.vel[1] *= d;
            motion.vel[2] *= d;
        }

        motion.status = rejected ? "gated" : "inertial";
    }

    motion.pos[0] += motion.vel[0] * dt + 0.5 * aUse[0] * dt * dt;
    motion.pos[1] += motion.vel[1] * dt + 0.5 * aUse[1] * dt * dt;
    motion.pos[2] += motion.vel[2] * dt + 0.5 * aUse[2] * dt * dt;

    const q = accQuality * (stationary ? 0.25 : 1.0);

    for (let i = 0; i < 3; i++) {
        const av = motion.noise[i] * motion.noise[i] / Math.max(0.05, q);
        motion.pVel[i] = Math.max(0.0, motion.pVel[i] + av * dt * dt);
        motion.pPos[i] = Math.max(0.0, motion.pPos[i] + motion.pVel[i] * dt * dt + 0.25 * av * dt * dt * dt * dt);
    }

    const posSigma = Math.sqrt(motion.pPos[0] + motion.pPos[1] + motion.pPos[2]);
    const biasMag = norm(motion.bias);
    motion.confidence = clamp(accQuality * Math.exp(-posSigma / 25.0) * Math.exp(-biasMag / 4.0), 0.0, 1.0);

    motion.lastAccelRaw = aRaw;
    motion.lastAccelUse = a;
    motion.lastR = R;
    motion.lastLaImuTimeUs = nowUs;
    motion.accepted += rejected ? 0 : 1;
    motion.path.push([...motion.pos]);
}

function integrateMotion() {
    if (!motionEnabled.checked) {
        motion.lastLaCount = latest.la_count;
        motion.lastLaImuTimeUs = latest.la_imu_time_us || motion.lastLaImuTimeUs;
        latest.la_samples = [];
        return;
    }

    const samples = Array.isArray(latest.la_samples) ? latest.la_samples : [];

    if (samples.length > 0) {
        for (const s of samples) {
            integrateLaSample(s[0], [s[1], s[2], s[3]], s[4], s[5]);
        }

        latest.la_samples = [];
        motion.lastLaCount = latest.la_count;
    } else if (latest.la !== null && latest.la_count !== motion.lastLaCount) {
        integrateLaSample(latest.la_imu_time_us, latest.la, latest.R, latest.accuracy);
        motion.lastLaCount = latest.la_count;
    }

    while (motion.path.length > 5000) {
        motion.path.shift();
    }
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

function drawUncertainty(scale) {
    const s = Math.sqrt(motion.pPos[0] + motion.pPos[1] + motion.pPos[2]) * scale;

    if (!Number.isFinite(s) || s <= 0.001) {
        return;
    }

    const center = scaleVec(motion.pos, scale);
    const pts = [];

    for (let i = 0; i <= 48; i++) {
        const a = 2.0 * Math.PI * i / 48.0;
        pts.push([center[0] + Math.cos(a) * s, center[1] + Math.sin(a) * s, center[2]]);
    }

    const pts2 = project(pts).map(screen);
    ctx.strokeStyle = "rgba(0,0,0,0.22)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(pts2[0][0], pts2[0][1]);

    for (let i = 1; i < pts2.length; i++) {
        ctx.lineTo(pts2[i][0], pts2[i][1]);
    }

    ctx.stroke();
}

function drawGroundGrid(scale) {
    const lim = 20;
    const pts = [];

    for (let i = -lim; i <= lim; i++) {
        pts.push([[i, -lim, 0], [i, lim, 0]]);
        pts.push([[-lim, i, 0], [lim, i, 0]]);
    }

    for (const g of pts) {
        const p = project([scaleVec(g[0], scale), scaleVec(g[1], scale)]).map(screen);
        line(p[0], p[1], "rgba(0,0,0,0.10)", 1);
    }
}

function draw() {
    updateCamera();
    integrateMotion();

    ctx.clearRect(0, 0, window.innerWidth, window.innerHeight);

    const R = getDisplayR();
    const motionScale = Number(motionScaleSlider.value);
    const center = scaleVec(motion.pos, motionScale);

    drawGroundGrid(motionScale);
    drawPath(motionScale);
    drawUncertainty(motionScale);

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

    const sigma = Math.sqrt(motion.pPos[0] + motion.pPos[1] + motion.pPos[2]);
    const rejectPct = motion.accepted + motion.rejected > 0 ? 100.0 * motion.rejected / (motion.accepted + motion.rejected) : 0.0;

    const text = [
        `RV count: ${latest.rv_count}`,
        `LA count: ${latest.la_count}`,
        `imu_time_us: ${latest.imu_time_us}`,
        `accuracy: ${Number(latest.accuracy).toFixed(3)}`,
        `zeroed: ${isZeroed ? "yes" : "no"}`,
        `model: ${motion.status}`,
        `pos m: ${motion.pos[0].toFixed(3)}, ${motion.pos[1].toFixed(3)}, ${motion.pos[2].toFixed(3)}`,
        `vel m/s: ${motion.vel[0].toFixed(3)}, ${motion.vel[1].toFixed(3)}, ${motion.vel[2].toFixed(3)}`,
        `bias m/s^2: ${motion.bias[0].toFixed(3)}, ${motion.bias[1].toFixed(3)}, ${motion.bias[2].toFixed(3)}`,
        `noise m/s^2: ${motion.noise[0].toFixed(3)}, ${motion.noise[1].toFixed(3)}, ${motion.noise[2].toFixed(3)}`,
        `angular rate rad/s: ${motion.angularRate.toFixed(3)}`,
        `jerk m/s^3: ${motion.jerk.toFixed(1)}`,
        `mahalanobis: ${motion.maha.toFixed(2)}`,
        `reject: ${rejectPct.toFixed(1)}%`,
        `pos sigma m: ${sigma.toFixed(3)}`,
        `confidence: ${(100.0 * motion.confidence).toFixed(1)}%`,
        `camera: ${camera.pos[0].toFixed(2)}, ${camera.pos[1].toFixed(2)}, ${camera.pos[2].toFixed(2)}`,
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

    setTimeout(poll, 1);
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
    const k = e.key.toLowerCase();
    camera.keys[k] = true;

    if (e.key === " " || k === "z") {
        zeroMatrix = transpose(latest.R);
        isZeroed = true;
        resetMotion();
    }

    if (k === "r") {
        resetMotion();
    }
});

window.addEventListener("keyup", e => {
    camera.keys[e.key.toLowerCase()] = false;
});

canvas.addEventListener("click", () => {
    canvas.requestPointerLock();
});

window.addEventListener("mousemove", e => {
    if (document.pointerLockElement !== canvas && e.buttons !== 1) {
        return;
    }

    camera.yaw += e.movementX * camera.mouseSensitivity;
    camera.pitch -= e.movementY * camera.mouseSensitivity;
    camera.pitch = Math.max(-1.45, Math.min(1.45, camera.pitch));
});

window.addEventListener("blur", () => {
    camera.keys = {};
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
                            R = quat_to_matrix(row["i"], row["j"], row["k"], row["real"]).tolist()
                            state["R"] = R
                            state["last_R"] = R
                            state["imu_time_us"] = row["imu_time_us"]
                            state["accuracy"] = row["accuracy"]
                            state["rv_count"] += 1
                            state["rv_host_time"] = row["host_time"]

                        if row["type"] == "LA":
                            R = state.get("last_R") or state.get("R")
                            accuracy = state.get("accuracy", 0.0)
                            sample = [row["imu_time_us"], row["x"], row["y"], row["z"], R, accuracy]
                            state["la"] = [row["x"], row["y"], row["z"]]
                            state["imu_time_us"] = row["imu_time_us"]
                            state["la_imu_time_us"] = row["imu_time_us"]
                            state["la_count"] += 1
                            state["la_host_time"] = row["host_time"]
                            state["la_samples"].append(sample)

                            if len(state["la_samples"]) > 4000:
                                del state["la_samples"][:-4000]

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
                    snapshot["la_samples"] = list(state.get("la_samples", []))
                    state["la_samples"] = []

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
        "last_R": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        "imu_time_us": 0,
        "accuracy": 0.0,
        "la": None,
        "rv_count": 0,
        "la_count": 0,
        "line": "",
        "la_host_time": 0.0,
        "rv_host_time": 0.0,
        "la_imu_time_us": 0,
        "la_samples": [],
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
