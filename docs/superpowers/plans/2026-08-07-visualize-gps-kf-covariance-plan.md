# GPS, Kalman Filter, and Covariance Visualization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reduce `visualize.py` to one overlaid GPS/KF path figure and one full covariance-trace figure.

**Architecture:** Keep the existing command-line CSV loading flow and calculate all plotted series directly from `csv.DictReader` rows. Create two independent Matplotlib figures: one with both position paths and one with the six-state covariance trace over elapsed time.

**Tech Stack:** Python 3 standard library and Matplotlib

## Global Constraints

- Modify the root `visualize.py`, not `microcontroller/visualize.py`.
- Treat `kf_easting_m` as longitude and `kf_northing_m` as latitude despite the CSV labels.
- Plot the covariance trace as the sum of all six diagonal covariance fields.
- Display exactly two graphs total.
- Add no comments to Python code.
- Preserve offline operation and add no dependencies.

---

### Task 1: Replace the Four-Panel Visualization

**Files:**
- Modify: `visualize.py:1-58`
- Test: transient noninteractive verification command; no test file added

**Interfaces:**
- Consumes: CSV path from the optional positional command-line argument, defaulting to `build/output.csv`
- Produces: one GPS/KF longitude-latitude figure and one elapsed-time covariance-trace figure

- [ ] **Step 1: Run a behavioral check that fails against the current four-panel output**

Run:

```bash
MPLBACKEND=Agg python -c 'import csv,runpy,sys; from pathlib import Path; import matplotlib.pyplot as plt; plt.show=lambda:None; sys.argv=["visualize.py","build/output.csv"]; runpy.run_path("visualize.py",run_name="__main__"); figures=[plt.figure(number) for number in plt.get_fignums()]; assert len(figures)==2; assert [len(figure.axes) for figure in figures]==[1,1]; path_axes,covariance_axes=[figure.axes[0] for figure in figures]; assert [line.get_label() for line in path_axes.lines]==["GPS","Kalman Filter"]; rows=list(csv.DictReader(Path("build/output.csv").open(newline=""))); expected=[sum(float(row[f"kf_covariance_{index}_{index}"]) for index in range(6)) for row in rows]; assert list(covariance_axes.lines[0].get_ydata())==expected'
```

Expected: FAIL because the current script creates one figure containing four axes.

- [ ] **Step 2: Implement exactly two figures in `visualize.py`**

Replace the KF speed and heading series with the covariance trace:

```python
kf_longitude = [float(row["kf_easting_m"]) for row in rows]
kf_latitude = [float(row["kf_northing_m"]) for row in rows]
covariance_trace = [
    sum(float(row[f"kf_covariance_{index}_{index}"]) for index in range(6))
    for row in rows
]
```

Create the combined path figure:

```python
path_figure, path_axes = plt.subplots(figsize=(10, 8))
path_axes.plot(gps_longitude, gps_latitude, label="GPS")
path_axes.plot(kf_longitude, kf_latitude, label="Kalman Filter")
path_axes.set_title(f"GPS and Kalman Filter Path — {csv_path.name}")
path_axes.set_xlabel("Longitude (degrees)")
path_axes.set_ylabel("Latitude (degrees)")
path_axes.axis("equal")
path_axes.grid(True)
path_axes.legend()
path_figure.tight_layout()
```

Create the covariance figure:

```python
covariance_figure, covariance_axes = plt.subplots(figsize=(10, 6))
covariance_axes.plot(elapsed_seconds, covariance_trace)
covariance_axes.set_title(f"Kalman Filter Covariance Trace — {csv_path.name}")
covariance_axes.set_xlabel("Elapsed time (s)")
covariance_axes.set_ylabel("Covariance trace")
covariance_axes.grid(True)
covariance_figure.tight_layout()
plt.show()
```

- [ ] **Step 3: Re-run the behavioral check**

Run the command from Step 1.

Expected: PASS with exit status 0, two figures, one axis per figure, both position series on the first axis, and the full covariance trace on the second.

- [ ] **Step 4: Verify syntax, formatting, scope, and the no-comments requirement**

Run:

```bash
python -m py_compile visualize.py
git diff --check -- visualize.py
git diff -- visualize.py
```

Expected: all commands exit successfully; the diff contains only the requested plotting change and contains no Python comments.

- [ ] **Step 5: Commit the implementation**

```bash
git add visualize.py
git commit -m "feat: simplify GPS KF visualization"
```
