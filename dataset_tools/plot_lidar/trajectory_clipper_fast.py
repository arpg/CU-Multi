
#!/usr/bin/env python3
"""
Multi-Robot Trajectory Clipper (PySide6 + pyqtgraph) — FAST

- Keeps your CSV loader exactly as-is (two next(f) + DictReader with '# timestamp',' x',' y').
- No command-line args required; still uses CU_MULTI_ROOT + fixed env/robots.
- Optimizations:
    * float32 arrays to halve memory bandwidth
    * searchsorted-based index slices (no boolean mask allocations)
    * pyqtgraph downsampling & clip-to-view on curves
    * Precise Qt timer and minimal per-tick math
"""

from __future__ import annotations
import os, sys, csv, math
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np
from PySide6 import QtCore, QtWidgets, QtGui
import pyqtgraph as pg


@dataclass
class Pose:
    t: float
    x: float
    y: float


# ---------- CSV loader (unchanged semantics) ----------
def load_csv(path: str) -> List[Pose]:
    pts: List[Pose] = []
    with open(path, "r", newline="") as f:
        next(f)
        next(f)

        reader = csv.DictReader(f)
        print(reader.fieldnames)
        if not {"# timestamp", " x", " y"}.issubset(reader.fieldnames or set()):
            raise ValueError("CSV must have headers: t,x,y")
        for row in reader:
            line_r = Pose(float(row['# timestamp']), float(row[' x']), float(row[' y']))
            print(f"pose_ts: {line_r}")
            try:
                pts.append(Pose(float(row["# timestamp"]), float(row[" x"]), float(row[" y"])))
            except Exception:
                continue
    pts.sort(key=lambda p: p.t)
    return pts


def demo_spiral(n=1200, dt=0.25, phase=0.0, jitter=0.0) -> List[Pose]:
    pts: List[Pose] = []
    t = 0.0
    for i in range(n):
        a = i * 0.05 + phase
        r = 2 + i * 0.01
        x = r * math.cos(a) + (np.random.randn() * jitter if jitter else 0.0)
        y = r * math.sin(a) + (np.random.randn() * jitter if jitter else 0.0)
        pts.append(Pose(t, x, y))
        t += dt
    return pts


def compute_speed(pts: List[Pose]) -> Tuple[np.ndarray, np.ndarray]:
    if len(pts) < 2:
        return np.array([p.t for p in pts], dtype=np.float32), np.zeros(len(pts), dtype=np.float32)
    t = np.array([p.t for p in pts], dtype=np.float32)
    x = np.array([p.x for p in pts], dtype=np.float32)
    y = np.array([p.y for p in pts], dtype=np.float32)
    dt = np.diff(t)
    dx = np.diff(x)
    dy = np.diff(y)
    dt[dt == 0] = np.finfo(np.float32).eps
    v = np.sqrt(dx * dx + dy * dy) / dt
    # Align sizes: define speed at segment midpoints; pad to t size
    t_mid = (t[:-1] + t[1:]) / np.float32(2.0)
    v_full = np.interp(t, t_mid, v, left=v[0] if len(v)>0 else 0.0, right=v[-1] if len(v)>0 else 0.0).astype(np.float32, copy=False)
    return t, v_full


class RobotPanel(QtWidgets.QWidget):
    """One timeline row: speed plot + its own region + scrub line + enable toggle."""
    regionChanged = QtCore.Signal(str, float, float)  # name, lo, hi
    scrubChanged = QtCore.Signal(str, float)          # name, scrub time

    def __init__(self, name: str, pts: List[Pose], color: Tuple[int,int,int]):
        super().__init__()
        self.name = name
        self.pts = pts
        self.color = color
        self.t = np.array([p.t for p in pts], dtype=np.float32)

        layout = QtWidgets.QVBoxLayout(self)
        hdr = QtWidgets.QHBoxLayout()
        self.enable_cb = QtWidgets.QCheckBox(name)
        self.enable_cb.setChecked(True)
        self.label = QtWidgets.QLabel("")
        self.label.setStyleSheet("color: #444;")
        hdr.addWidget(self.enable_cb)
        hdr.addStretch(1)
        hdr.addWidget(self.label)
        layout.addLayout(hdr)

        self.plot = pg.PlotWidget()
        self.plot.setBackground("w")
        self.plot.showGrid(x=True, y=True, alpha=0.15)
        self.plot.setLabel("bottom", f"Time (s) — {name}")
        self.plot.setLabel("left", "Speed")
        layout.addWidget(self.plot, stretch=1)

        t, v = compute_speed(self.pts)
        pen = pg.mkPen(color, width=1.4)
        self.curve = self.plot.plot(t, v, pen=pen)
        self.curve.setDownsampling(auto=True, method='peak')
        self.curve.setClipToView(True)

        if len(self.t) >= 2:
            lo = float(np.quantile(self.t, 0.10))
            hi = float(np.quantile(self.t, 0.60))
        else:
            lo = float(self.t[0]) if len(self.t) else 0.0
            hi = float(self.t[-1]) if len(self.t) else 1.0

        # Region + scrub
        self.region = pg.LinearRegionItem(values=(lo, hi), brush=pg.mkBrush(*color, 40), movable=True)
        self.region.setZValue(-10)
        self.plot.addItem(self.region)

        self.scrub = pg.InfiniteLine(pos=lo, angle=90, movable=True, pen=pg.mkPen(color, width=2))
        self.plot.addItem(self.scrub)

        # Signals
        self.region.sigRegionChanged.connect(self._region_changed)
        self.scrub.sigPositionChanged.connect(self._scrub_changed)
        self.enable_cb.toggled.connect(self._region_changed)  # update label

        self._update_label()

    def _region_changed(self):
        lo, hi = self.region.getRegion()
        # keep scrub inside region
        s = float(self.scrub.value())
        if s < lo:
            self.scrub.setValue(lo)
        if s > hi:
            self.scrub.setValue(hi)
        self.regionChanged.emit(self.name, lo, hi)
        self._update_label()

    def _scrub_changed(self):
        self.scrubChanged.emit(self.name, float(self.scrub.value()))
        self._update_label()

    def _update_label(self):
        lo, hi = self.region.getRegion()
        s = float(self.scrub.value())
        state = "ON" if self.enable_cb.isChecked() else "OFF"
        self.label.setText(f"{state} | Window {lo:.2f}–{hi:.2f}s | Scrub {s:.2f}s")

    # Convenience getters
    def is_enabled(self) -> bool:
        return self.enable_cb.isChecked()

    def get_window(self) -> Tuple[float, float]:
        return self.region.getRegion()

    def set_scrub(self, tval: float):
        lo, hi = self.region.getRegion()
        tval = min(max(tval, lo), hi)
        self.scrub.setValue(tval)


class MultiTrajectoryClipper(QtWidgets.QMainWindow):
    def __init__(self, robots: Dict[str, List[Pose]], colors: Dict[str, Tuple[int,int,int]]):
        super().__init__()
        self.setWindowTitle("Multi-Robot Trajectory Clipper — FAST")
        self.resize(1300, 900)

        self.robots = robots
        self.colors = colors

        # Precompute arrays (float32 to cut memory/bandwidth)
        self.arr = {name: (np.asarray([p.t for p in pts], dtype=np.float32),
                           np.asarray([p.x for p in pts], dtype=np.float32),
                           np.asarray([p.y for p in pts], dtype=np.float32))
                    for name, pts in robots.items()}

        # --- UI ---
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        vbox = QtWidgets.QVBoxLayout(central)

        # Top XY plot (combined)
        pg.setConfigOptions(antialias=True)
        self.xy = pg.PlotWidget()
        self.xy.setBackground("w")
        self.xy.showGrid(x=True, y=True, alpha=0.15)
        self.xy.setAspectLocked(True, ratio=1)
        self.xy.setLabel("left", "Y")
        self.xy.setLabel("bottom", "X")
        vbox.addWidget(self.xy, stretch=5)

        # Draw full paths (light) and clipped overlays per robot
        self.curves_full = {}
        self.curves_clip = {}
        self.scrub_points = {}

        for name, (t, x, y) in self.arr.items():
            col = self.colors[name]
            light = tuple(max(0, min(255, int(0.55*c))) for c in col)

            full_curve = self.xy.plot(x, y, pen=pg.mkPen(light, width=2))
            full_curve.setDownsampling(auto=True, method='peak')
            full_curve.setClipToView(True)
            self.curves_full[name] = full_curve

            clip_curve = self.xy.plot([], [], pen=pg.mkPen(col, width=3))
            clip_curve.setDownsampling(auto=True, method='peak')
            clip_curve.setClipToView(True)
            self.curves_clip[name] = clip_curve

            self.scrub_points[name] = pg.ScatterPlotItem(size=8, brush=pg.mkBrush(*col), pen=pg.mkPen(None))
            self.xy.addItem(self.scrub_points[name])

        # Controls row
        ctrl = QtWidgets.QHBoxLayout()
        vbox.addLayout(ctrl)
        self.play_btn = QtWidgets.QPushButton("Play All")
        self.pause_btn = QtWidgets.QPushButton("Pause All")
        ctrl.addWidget(self.play_btn)
        ctrl.addWidget(self.pause_btn)
        ctrl.addStretch(1)

        # Scroll area for per-robot timelines
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        vbox.addWidget(scroll, stretch=5)
        inner = QtWidgets.QWidget()
        scroll.setWidget(inner)
        self.timeline_layout = QtWidgets.QVBoxLayout(inner)

        self.panels: Dict[str, RobotPanel] = {}
        for name, pts in robots.items():
            col = self.colors[name]
            panel = RobotPanel(name, pts, col)
            panel.regionChanged.connect(self._on_region_changed)
            panel.scrubChanged.connect(self._on_scrub_changed)
            self.timeline_layout.addWidget(panel)
            self.panels[name] = panel

        self.timeline_layout.addStretch(1)

        # Timer for global playback (advances each panel's scrub if enabled)
        self.timer = QtCore.QTimer(self)
        self.timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.timer.setInterval(33)  # ~30 FPS; raise to 16 for ~60 FPS if needed
        self.timer.timeout.connect(self._tick)

        self.play_btn.clicked.connect(self._on_play_all)
        self.pause_btn.clicked.connect(self._on_pause_all)

        # Shortcuts
        QtGui.QShortcut(QtGui.QKeySequence("Space"), self, activated=self._toggle_play_pause)

        # Initial update
        self._update_all_clips()
        self._update_all_scrubs()

    # ---- Handlers ----
    def _on_region_changed(self, name: str, lo: float, hi: float):
        self._update_clip(name)

    def _on_scrub_changed(self, name: str, tval: float):
        self._update_scrub_point(name)

    def _on_play_all(self):
        if not self.timer.isActive():
            self.timer.start()

    def _on_pause_all(self):
        if self.timer.isActive():
            self.timer.stop()

    def _toggle_play_pause(self):
        if self.timer.isActive():
            self._on_pause_all()
        else:
            self._on_play_all()

    # ---- Updates ----
    def _update_clip(self, name: str):
        t, x, y = self.arr[name]
        lo, hi = self.panels[name].get_window()
        # Use index slice instead of boolean mask to avoid allocations
        i0 = int(np.searchsorted(t, np.float32(lo), side="left"))
        i1 = int(np.searchsorted(t, np.float32(hi), side="right"))
        if i1 - i0 <= 1:
            self.curves_clip[name].setData([], [])
            return
        self.curves_clip[name].setData(x[i0:i1], y[i0:i1])

    def _update_all_clips(self):
        for name in self.robots.keys():
            self._update_clip(name)

    def _update_scrub_point(self, name: str):
        t, x, y = self.arr[name]
        s = np.float32(self.panels[name].scrub.value())
        if t.size == 0:
            return
        idx = int(np.searchsorted(t, s))
        if idx >= t.size:
            idx = t.size - 1
        self.scrub_points[name].setData([float(x[idx])], [float(y[idx])])

    def _update_all_scrubs(self):
        for name in self.robots.keys():
            self._update_scrub_point(name)

    def _tick(self):
        # Advance each enabled robot's scrubber within its own region
        dt = self.timer.interval() * 0.001  # seconds
        step = 1.0 * dt                      # adjust speed scalar if desired
        for name, panel in self.panels.items():
            if not panel.is_enabled():
                continue
            lo, hi = panel.get_window()
            s = float(panel.scrub.value()) + step
            if s > hi:
                s = lo
            panel.set_scrub(s)
            self._update_scrub_point(name)


def color_cycle(n: int) -> List[Tuple[int,int,int]]:
    base = [
        (87, 227, 137),
        (192, 97, 203),
        (255, 163, 72),
        (98, 160, 234),
        (255, 99, 132),
        (255, 206, 86),
        (75, 192, 192),
        (153, 102, 255),
    ]
    out = []
    i = 0
    while len(out) < n:
        out.append(base[i % len(base)])
        i += 1
    return out


def main():
    DATA_ROOT = os.getenv("CU_MULTI_ROOT")
    if DATA_ROOT is None:
        sys.exit(
            "ERROR: Environment variable CU_MULTI_ROOT is not set.\n"
            "Please set it before running the script, e.g.:\n"
            "  export CU_MULTI_ROOT=/your/custom/path\n"
        )
    print("CU_MULTI_ROOT is:", DATA_ROOT)

    env = "kittredge_loop"
    robotIDs = [1, 2, 3, 4]

    robots: Dict[str, List[Pose]] = {}
    colors: Dict[str, Tuple[int,int,int]] = {}

    root = os.getenv("CU_MULTI_ROOT")
    if not root:
        sys.exit("Set CU_MULTI_ROOT.")

    for robotID in robotIDs:
        name = f"{env}_r{robotID}"
        p = os.path.join(root, "gt_poses", f"{env}_robot{robotID}_ref.csv")
        pts = load_csv(p)
        robots[name] = pts
        print(f"Loaded {name}: {len(pts)} poses")

    cols = color_cycle(len(robots))
    for name, col in zip(robots.keys(), cols):
        colors[name] = col

    app = QtWidgets.QApplication(sys.argv)
    win = MultiTrajectoryClipper(robots, colors)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
