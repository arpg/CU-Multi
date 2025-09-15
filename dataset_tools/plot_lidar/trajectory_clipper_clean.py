#!/usr/bin/env python3
"""
Multi-Robot Trajectory Clipper (PySide6 + pyqtgraph) — CLEAN

- No full trajectory lines (only clipped segments + scrub points).
- Timelines locked to each robot's time span (no scrolling past begin/end).
- Timeline plots hide bottom axis labels (only region + scrub visible).
- CSV loader preserved as-is (two next(f) calls).
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
            try:
                pts.append(Pose(float(row["# timestamp"]), float(row[" x"]), float(row[" y"])))
            except Exception:
                continue
    pts.sort(key=lambda p: p.t)
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
    t_mid = (t[:-1] + t[1:]) / np.float32(2.0)
    v_full = np.interp(t, t_mid, v, left=v[0] if len(v)>0 else 0.0, right=v[-1] if len(v)>0 else 0.0).astype(np.float32, copy=False)
    return t, v_full


class RobotPanel(QtWidgets.QWidget):
    regionChanged = QtCore.Signal(str, float, float)
    scrubChanged = QtCore.Signal(str, float)

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
        self.plot.setLabel("left", "Speed")
        self.plot.hideAxis("bottom")
        layout.addWidget(self.plot, stretch=1)

        t, v = compute_speed(self.pts)
        pen = pg.mkPen(color, width=1.4)
        self.curve = self.plot.plot(t, v, pen=pen)
        self.curve.setDownsampling(auto=True, method='peak')
        self.curve.setClipToView(True)

        if len(self.t) >= 2:
            lo = float(self.t[0])
            hi = float(self.t[-1])
        else:
            lo, hi = 0.0, 1.0

        self.plot.setXRange(lo, hi, padding=0)

        self.region = pg.LinearRegionItem(values=(lo, hi), brush=pg.mkBrush(*color, 40), movable=True)
        self.region.setZValue(-10)
        self.plot.addItem(self.region)

        self.scrub = pg.InfiniteLine(pos=lo, angle=90, movable=True, pen=pg.mkPen(color, width=2))
        self.plot.addItem(self.scrub)

        self.region.sigRegionChanged.connect(self._region_changed)
        self.scrub.sigPositionChanged.connect(self._scrub_changed)
        self.enable_cb.toggled.connect(self._region_changed)

        self._update_label()

    def _region_changed(self):
        lo, hi = self.region.getRegion()
        s = float(self.scrub.value())
        if s < lo: self.scrub.setValue(lo)
        if s > hi: self.scrub.setValue(hi)
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
        self.setWindowTitle("Multi-Robot Trajectory Clipper Tool (Clean)")
        self.resize(1300, 900)

        self.robots = robots
        self.colors = colors

        self.arr = {name: (np.asarray([p.t for p in pts], dtype=np.float32),
                           np.asarray([p.x for p in pts], dtype=np.float32),
                           np.asarray([p.y for p in pts], dtype=np.float32))
                    for name, pts in robots.items()}

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        vbox = QtWidgets.QVBoxLayout(central)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        vbox.addWidget(splitter)

        self.xy = pg.PlotWidget()
        self.xy.setBackground("w")
        self.xy.showGrid(x=True, y=True, alpha=0.15)
        self.xy.setAspectLocked(True, ratio=1)
        self.xy.setLabel("left", "Y")
        self.xy.setLabel("bottom", "X")
        splitter.addWidget(self.xy)

        bottom = QtWidgets.QWidget()
        bottom_v = QtWidgets.QVBoxLayout(bottom)
        splitter.addWidget(bottom)

        ctrl = QtWidgets.QHBoxLayout()
        self.play_btn = QtWidgets.QPushButton("Play All")
        self.pause_btn = QtWidgets.QPushButton("Pause All")
        ctrl.addWidget(self.play_btn)
        ctrl.addWidget(self.pause_btn)
        ctrl.addStretch(1)
        bottom_v.addLayout(ctrl)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        bottom_v.addWidget(scroll, stretch=1)
        inner = QtWidgets.QWidget()
        scroll.setWidget(inner)
        self.timeline_layout = QtWidgets.QVBoxLayout(inner)

        self.curves_clip = {}
        self.scrub_points = {}

        for name, (t, x, y) in self.arr.items():
            col = self.colors[name]
            clip_curve = self.xy.plot([], [], pen=pg.mkPen(col, width=3))
            clip_curve.setDownsampling(auto=True, method='peak')
            clip_curve.setClipToView(True)
            self.curves_clip[name] = clip_curve

            self.scrub_points[name] = pg.ScatterPlotItem(size=8, brush=pg.mkBrush(*col), pen=pg.mkPen(None))
            self.xy.addItem(self.scrub_points[name])

        self.panels: Dict[str, RobotPanel] = {}
        for name, pts in robots.items():
            col = self.colors[name]
            panel = RobotPanel(name, pts, col)
            panel.regionChanged.connect(self._on_region_changed)
            panel.scrubChanged.connect(self._on_scrub_changed)
            self.timeline_layout.addWidget(panel)
            self.panels[name] = panel
            panel.enable_cb.toggled.connect(lambda checked, n=name: self._set_robot_visible(n, checked))

        self.timeline_layout.addStretch(1)

        self.timer = QtCore.QTimer(self)
        self.timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.timer.setInterval(33)
        self.timer.timeout.connect(self._tick)

        self.play_btn.clicked.connect(self._on_play_all)
        self.pause_btn.clicked.connect(self._on_pause_all)

        QtGui.QShortcut(QtGui.QKeySequence("Space"), self, activated=self._toggle_play_pause)

        self._update_all_clips()
        self._update_all_scrubs()

    def _set_robot_visible(self, name: str, visible: bool):
        self.curves_clip[name].setVisible(visible)
        self.scrub_points[name].setVisible(visible)

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

    def _update_clip(self, name: str):
        if not self.panels[name].is_enabled():
            self.curves_clip[name].setVisible(False)
            return
        self.curves_clip[name].setVisible(True)
        t, x, y = self.arr[name]
        lo, hi = self.panels[name].get_window()
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
        if not self.panels[name].is_enabled():
            self.scrub_points[name].setVisible(False)
            return
        self.scrub_points[name].setVisible(True)
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
        dt = self.timer.interval() * 0.001
        step = 1.0 * dt
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
        sys.exit("ERROR: CU_MULTI_ROOT not set.")

    env = "kittredge_loop"
    robotIDs = [1, 2, 3, 4]

    robots: Dict[str, List[Pose]] = {}
    colors: Dict[str, Tuple[int,int,int]] = {}

    for robotID in robotIDs:
        name = f"{env}_r{robotID}"
        p = os.path.join(DATA_ROOT, "gt_poses", f"{env}_robot{robotID}_ref.csv")
        pts = load_csv(p)
        robots[name] = pts
        print(f"Loaded {name}: {len(pts)} poses")

    cols = color_cycle(len(robots))
    for name, col in zip(robots.keys(), cols):
        colors[name] = col

    app = QtWidgets.QApplication(sys.argv)
    app.setStyleSheet("QWidget { background-color: white; }")
    win = MultiTrajectoryClipper(robots, colors)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
