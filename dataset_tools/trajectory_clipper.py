#!/usr/bin/env python3
"""
CU-Multi ROS2 Trajectory Clipper

tunable variables (top of file):
  * MAX_DT_GAP_S      : split segments if time gap exceeds this
  * MAX_DIST_GAP_M    : split if spatial jump exceeds this (set 0 to disable)
  * SEG_PEN_WIDTH     : polyline width in the XY view
  * MAX_SEGS          : number of preallocated faded segments per robot
"""

from __future__ import annotations
import os, sys, csv
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np
from PySide6 import QtCore, QtWidgets, QtGui
import pyqtgraph as pg

import clip_ros2_bags

# ---- rendering & continuity controls ----
DTYPE           = np.float64
MAX_SEGS        = 128    # preallocated faded segments per robot
SEG_PEN_WIDTH   = 8     # thicker lines for visibility
MAX_DT_GAP_S    = 1    # split if time gap between neighbors > this
MAX_DIST_GAP_M  = 0.5   # split if spatial jump > this (0.0 => disable distance check)
MIN_ALPHA = 100
MAX_ALPHA = 255

SNAP_MAX_DT_S = 0.5   # only snap if a sample is within 0.5 s (tweak)

@dataclass
class Pose:
    t: float
    x: float
    y: float


# ---------- csv loader (unchanged semantics) ----------
def load_csv(path: str) -> List[Pose]:
    pts: List[Pose] = []
    with open(path, "r", newline="") as f:
        next(f)
        next(f)
        reader = csv.DictReader(f)
        if not {"# timestamp", " x", " y"}.issubset(reader.fieldnames or set()):
            raise ValueError("CSV must have headers: '# timestamp',' x',' y'")
        for row in reader:
            try:
                pts.append(Pose(float(row["# timestamp"]), float(row[" x"]), float(row[" y"])))
            except Exception:
                continue
    pts.sort(key=lambda p: p.t)
    return pts


def linear_curve(t: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Simple 0..1 curve over t (assumes t starts at 0)."""
    if t.size < 2:
        tt = np.array([0.0, 1.0], dtype=DTYPE)
        return tt, tt.copy()
    if t[-1] <= 0:
        return t, np.zeros_like(t)
    y = t / max(1e-9, t[-1])
    return t, y.astype(DTYPE, copy=False)


class MultiTrajectoryClipper(QtWidgets.QMainWindow):
    def __init__(self, robots: Dict[str, List[Pose]], colors: Dict[str, Tuple[int,int,int]]):
        super().__init__()
        self.setWindowTitle("CU-Multi Trajectory Clipper")
        self.resize(1300, 900)

        self.colors = colors

        # shift to t0 = 0 per robot, but keep absolute t0 for export
        self.arr: Dict[str, Tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
        self.durations: Dict[str, float] = {}
        self.t0_abs: Dict[str, float] = {}  # absolute first timestamp per robot
        for name, pts in robots.items():
            t_abs = np.asarray([p.t for p in pts], dtype=DTYPE)
            if t_abs.size:
                self.t0_abs[name] = float(t_abs[0])
                t = t_abs - t_abs[0]
            else:
                self.t0_abs[name] = 0.0
                t = t_abs
            x = np.asarray([p.x for p in pts], dtype=DTYPE)
            y = np.asarray([p.y for p in pts], dtype=DTYPE)
            self.arr[name] = (t, x, y)
            self.durations[name] = float(t[-1]) if t.size else 0.0

        self.global_max = max(self.durations.values()) if self.durations else 1.0

        # per-robot windows (zero-shifted seconds)
        self.windows: Dict[str, Tuple[float, float]] = {}
        for name, dur in self.durations.items():
            if dur > 0:
                self.windows[name] = (0.10 * dur, 0.60 * dur)
            else:
                self.windows[name] = (0.0, 0.0)

        # ---- UI ----
        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        vbox = QtWidgets.QVBoxLayout(central)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical); splitter.setChildrenCollapsible(False)
        vbox.addWidget(splitter)

        # XY plot (just trajectories)
        pg.setConfigOptions(antialias=True)
        self.xy = pg.PlotWidget()
        self.xy.setBackground("w")
        self.xy.hideAxis("bottom"); self.xy.hideAxis("left")
        self.xy.showGrid(x=False, y=False)
        self.xy.setAspectLocked(True, ratio=1)
        splitter.addWidget(self.xy)

        # bottom controls + single timeline
        bottom = QtWidgets.QWidget(); bottom_v = QtWidgets.QVBoxLayout(bottom)
        bottom_v.setContentsMargins(6, 6, 6, 6)
        splitter.addWidget(bottom)

        ctrl = QtWidgets.QHBoxLayout()
        ctrl.addWidget(QtWidgets.QLabel("Robot:"))
        self.robot_sel = QtWidgets.QComboBox()
        for name in self.arr.keys(): self.robot_sel.addItem(name)
        ctrl.addWidget(self.robot_sel)

        self.report_btn = QtWidgets.QPushButton("Report Clip Range")
        # self.report_all_btn = QtWidgets.QPushButton("Report Robot")
        self.export_unix_btn = QtWidgets.QPushButton("Retime ROS2 bags")
        # ctrl.addWidget(self.report_btn)
        # ctrl.addWidget(self.report_all_btn)
        ctrl.addWidget(self.export_unix_btn)
        ctrl.addStretch(1)
        bottom_v.addLayout(ctrl)

        # Single timeline
        self.timeline = pg.PlotWidget()
        self.timeline.setBackground("w")
        self.timeline.hideAxis("bottom")
        self.timeline.hideAxis("left")
        self.timeline.setMouseEnabled(x=False, y=False)
        bottom_v.addWidget(self.timeline, stretch=1)

        # Background timelines & faint windows
        self.bg_curves: Dict[str, pg.PlotDataItem] = {}
        self.bg_regions: Dict[str, pg.LinearRegionItem] = {}
        for name in self.arr.keys():
            t, _, _ = self.arr[name]
            tt, yy = linear_curve(t if t.size else np.array([0,1], dtype=DTYPE))
            col = self.colors[name]
            bg = self.timeline.plot(tt, yy, pen=pg.mkPen(col[0], col[1], col[2], 70, width=1.0))
            bg.setZValue(-20)
            self.bg_curves[name] = bg

            lo, hi = self.windows[name]
            region = pg.LinearRegionItem(values=(lo, hi), bounds=[0.0, max(1.0, self.durations[name])], movable=False)
            region.setBrush(pg.mkBrush(col[0], col[1], col[2], 28))
            for ln in region.lines:
                ln.setPen(pg.mkPen(col[0], col[1], col[2], 80, width=1))
            region.setZValue(-15)
            self.timeline.addItem(region)
            self.bg_regions[name] = region

        # global black baseline time curve
        t_global = np.linspace(0.0, self.global_max if self.global_max > 0 else 1.0, num=200, dtype=DTYPE)
        tt_g, yy_g = linear_curve(t_global)
        self.curve_global = self.timeline.plot(tt_g, yy_g, pen=pg.mkPen(0, 0, 0, 130, width=1.0))
        self.curve_global.setZValue(-10)

        # Selected robot curve (colored)
        self.curve_robot = self.timeline.plot([], [], pen=pg.mkPen(0, 0, 0, 180, width=1.8))
        self.curve_robot.setZValue(-5)

        # Editable region (selected robot only)
        self.region = pg.LinearRegionItem(values=(0.0, 0.0), bounds=[0.0, max(1.0, self.global_max)], movable=True)
        self.timeline.addItem(self.region)

        # Lock ranges
        vb = self.timeline.getViewBox()
        vb.setLimits(xMin=0.0, xMax=self.global_max, minXRange=self.global_max, maxXRange=self.global_max)
        self.timeline.setXRange(0.0, self.global_max, padding=0.0)
        self.timeline.setYRange(0.0, 1.0, padding=0.0)

        # XY preallocated items
        self.segments: Dict[str, List[pg.PlotDataItem]] = {}
        self.start_markers: Dict[str, pg.ScatterPlotItem] = {}
        self.end_markers: Dict[str, pg.ScatterPlotItem] = {}
        for name, (t, x, y) in self.arr.items():
            col = self.colors[name]
            segs: List[pg.PlotDataItem] = []
            for k in range(MAX_SEGS):
                alpha = int(MIN_ALPHA + 190 * (k / max(1, MAX_SEGS - 1)))  # 40..230 across time
                pen = pg.mkPen(col[0], col[1], col[2], alpha, width=SEG_PEN_WIDTH)
                item = self.xy.plot([], [], pen=pen, clear=False)
                item.setClipToView(True)
                item.setDownsampling(auto=False, method='peak')
                segs.append(item)
            self.segments[name] = segs
            sp_start = pg.ScatterPlotItem(size=9, brush=pg.mkBrush(col[0], col[1], col[2], 155), pen=pg.mkPen(None))
            sp_end   = pg.ScatterPlotItem(size=11, brush=pg.mkBrush(col[0], col[1], col[2], 255), pen=pg.mkPen(30,30,30,30))
            self.xy.addItem(sp_start); self.xy.addItem(sp_end)
            self.start_markers[name] = sp_start; self.end_markers[name] = sp_end

        # Events
        self._switching = False
        self.robot_sel.currentTextChanged.connect(self._on_robot_changed)
        self.region.sigRegionChanged.connect(self._on_region_changed)
        if hasattr(self.region, "sigRegionChangeFinished"):
            self.region.sigRegionChangeFinished.connect(self._on_region_change_finished)

        self.report_btn.clicked.connect(self._on_report)
        # self.report_all_btn.clicked.connect(self._on_report_all)
        self.export_unix_btn.clicked.connect(self._on_clip_ros2_bags)

        # Initial render
        first = self.robot_sel.currentText() if self.robot_sel.count() else None
        self._on_robot_changed(first)
        self._update_all_xy()

    # # ---- Helpers for snapping and indices ----
    # def _snap_to_sample(self, name: str, t_val: float) -> float:
    #     t, _, _ = self.arr[name]
    #     if t.size == 0:
    #         return 0.0
    #     j = int(np.searchsorted(t, DTYPE(t_val)))
    #     j = max(0, min(j, t.size-1))
    #     if j > 0 and abs(t_val - float(t[j-1])) < abs(t_val - float(t[j])):
    #         j -= 1
    #     return float(t[j])

    def _snap_to_sample(self, name: str, t_val: float) -> float:
        """Snap to nearest sample only if it's close and not across a big gap."""
        t, _, _ = self.arr[name]
        if t.size == 0:
            return 0.0

        j = int(np.searchsorted(t, t_val))
        j = max(0, min(j, t.size - 1))

        # pick nearest of (j-1, j)
        cand = [j]
        if j > 0: cand.append(j - 1)
        # compute distances
        dt_candidates = [(abs(t_val - float(t[k])), k) for k in cand]
        dt_min, k_best = min(dt_candidates, key=lambda z: z[0])

        # respect large time gaps (MAX_DT_GAP_S). If we chose k_best next to a big gap and
        # the other side is closer in time, prefer the closer one—but only if within SNAP_MAX_DT_S.
        if dt_min > SNAP_MAX_DT_S:
            # don't snap if nothing is close enough
            return t_val

        return float(t[k_best])


    def _idx_from_time(self, t: np.ndarray, val: float) -> int:
        j = int(np.searchsorted(t, DTYPE(val)))
        return max(0, min(j, t.size-1))

    # ---- Reporting ----
    def _on_report(self):
        if not self.windows:
            QtWidgets.QMessageBox.information(self, "Clip Range", "No windows set.")
            return
        los = [w[0] for w in self.windows.values()]
        his = [w[1] for w in self.windows.values()]
        earliest = float(min(los)) if los else 0.0
        latest   = float(max(his)) if his else 0.0
        msg = f"Earliest start (zero-shifted): {earliest:.3f} s\nLatest end (zero-shifted): {latest:.3f} s"
        print(msg)
        QtWidgets.QMessageBox.information(self, "Clip Range", msg)

    def _on_report_all(self):
        lines = []
        lines.append("Per-Robot Clip Windows (zero-shifted seconds)")
        lines.append("Robot, start_s, end_s, span_s, duration_s, trim_before_s, trim_after_s")
        for name, (lo, hi) in self.windows.items():
            dur = self.durations.get(name, 0.0)
            span = max(0.0, hi - lo)
            trim_before = max(0.0, lo)
            trim_after  = max(0.0, dur - hi)
            lines.append(f"{name}, {lo:.3f}, {hi:.3f}, {span:.3f}, {dur:.3f}, {trim_before:.3f}, {trim_after:.3f}")
        text = "\n".join(lines)
        print(text)
        QtWidgets.QMessageBox.information(self, "Per-Robot Windows", text)

    def _create_robot_times_dict(self):
        # out: Dict[str, DecimalPair] = {}

        # for row in csv_rdr:
        #     robot_name = str(row[robot_names]).strip()
        #     start_time = Decimal(row[start_times].strip())
        #     end_time = Decimal(row[end_times].strip())
        #     out[robot_name] = (start_time, end_time)
        # return out
        return 0.0

    def _on_clip_ros2_bags(self):
        from decimal import Decimal, getcontext
        DecimalPair = Tuple[Decimal, Decimal]

        out: Dict[str, DecimalPair] = {}
        # earliest = None
        # latest = None

        for name in self.arr.keys():
            times = self.windows.get(name)

            t0 = self.t0_abs.get(name)
            start_unix = float(t0 + times[0])
            end_unix = float(t0 + times[1])

            start_time = Decimal(start_unix)
            end_time = Decimal(end_unix)
            out[name] = (start_time, end_time)
        
            # if earliest is None or start_unix < earliest: 
            #     earliest = start_unix
            # if latest   is None or end_unix > latest:   
            #     latest   = end_unix
            print(f"name: {name}, start_unix: {start_unix}")

        clip_ros2_bags.clip_robot_bags(out)


    def _on_export_unix_csv(self):
        """Export CSV with absolute UNIX times:
           unix_start_ts = t0_abs[name] + lo
           unix_end_ts   = t0_abs[name] + hi
        """
        if not self.windows:
            QtWidgets.QMessageBox.information(self, "Export", "No windows set.")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save UNIX Windows CSV",
                                                        "robot_windows_unix.csv",
                                                        "CSV files (*.csv)")
        if not path:
            return
        earliest = None; latest = None
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["robot", "unix_start_ts", "unix_end_ts"])
            for name in self.arr.keys():
                lo, hi = self.windows.get(name, (0.0, 0.0))
                t0 = self.t0_abs.get(name, 0.0)
                s_abs = float(t0 + lo)
                e_abs = float(t0 + hi)

                if earliest is None or s_abs < earliest: 
                    earliest = s_abs
                if latest   is None or e_abs > latest:   
                    latest   = e_abs

                w.writerow([name, f"{s_abs:.9f}", f"{e_abs:.9f}"])
        msg = f"Saved UNIX windows CSV:\n{path}\nGlobal range: [{earliest:.9f}, {latest:.9f}]"
        print(msg)
        QtWidgets.QMessageBox.information(self, "Export", msg)

    # ---- selection / region ----
    def _on_robot_changed(self, name: str | None):
        if not name:
            return
        self._switching = True
        try:
            # Selected colored curve
            t, _, _ = self.arr[name]
            tt, yy = linear_curve(t if t.size else np.array([0,1], dtype=DTYPE))
            self.curve_robot.setData(tt, yy)
            col = self.colors[name]
            self.curve_robot.setPen(pg.mkPen(*col, 220, width=10))

            # Update editable region bounds and value
            dur = self.durations[name]
            self.region.setBounds([0.0, max(1e-9, dur)])
            lo, hi = self.windows[name]
            lo = max(0.0, min(lo, dur)); hi = max(lo, min(hi, dur)) if dur > 0 else 0.0
            self.region.blockSignals(True); self.region.setRegion((lo, hi)); self.region.blockSignals(False)

            # Recolor region
            fill = pg.mkBrush(col[0], col[1], col[2], 125)
            edge = pg.mkPen(col, width=10)
            self.region.setBrush(fill); [line.setPen(edge) for line in self.region.lines]
        finally:
            self._switching = False

    def _on_region_changed(self):
        if self._switching:
            return
        
        name = self.robot_sel.currentText()
        if not name:
            return
        
        lo, hi = self.region.getRegion()
        self.windows[name] = (float(lo), float(hi))
        bg = self.bg_regions.get(name)
        
        if bg is not None:
            bg.setRegion((float(lo), float(hi)))
        
        self._update_robot_xy(name)

    def _on_region_change_finished(self):
        """Snap region edges to the nearest samples of the selected robot after drag."""
        name = self.robot_sel.currentText()

        if not name:
            return
        
        lo, hi = self.region.getRegion()
        lo_s = self._snap_to_sample(name, lo)
        hi_s = self._snap_to_sample(name, hi)

        if hi_s < lo_s:
            hi_s = lo_s

        self.region.blockSignals(True)
        self.region.setRegion((lo_s, hi_s))
        self.region.blockSignals(False)

        self.windows[name] = (lo_s, hi_s)
        bg = self.bg_regions.get(name)
        
        if bg is not None:
            bg.setRegion((lo_s, hi_s))
        
        self._update_robot_xy(name)

    def _shift_window(self, samples: int, big: bool):
        name = self.robot_sel.currentText()
        if not name: return
        t, _, _ = self.arr[name]
        lo, hi = self.windows[name]
        i0 = self._idx_from_time(t, lo)
        i1 = self._idx_from_time(t, hi)
        i0 = max(0, min(i0 + samples, t.size-1))
        i1 = max(i0, min(i1 + samples, t.size-1))
        lo_s = float(t[i0]); hi_s = float(t[i1])
        self.region.blockSignals(True); self.region.setRegion((lo_s, hi_s)); self.region.blockSignals(False)
        self.windows[name] = (lo_s, hi_s)
        bg = self.bg_regions.get(name)
        if bg is not None:
            bg.setRegion((lo_s, hi_s))
        self._update_robot_xy(name)


    def _set_alpha_ramp_for_used(self, name: str, used: int):
        """Re-ink pens for the first `used` segments so alpha ramps from head->tail."""
        col = self.colors[name]
        segs = self.segments[name]
        if used <= 0:
            return
        # ramp MIN_ALPHA .. MAX_ALPHA across the *actually used* segments
        for k in range(used):
            a = int(np.interp(k, [0, max(1, used - 1)], [MIN_ALPHA, MAX_ALPHA]))
            segs[k].setPen(pg.mkPen(col[0], col[1], col[2], a, width=SEG_PEN_WIDTH))

    def _normalize_runs(self, runs, target_min, target_max):
        """
        runs: list of (s,e) index pairs (exclusive end), each with length >= 2
        Returns a new list of runs whose count is clamped into [target_min, target_max]
        by merging or subdividing evenly (purely visual).
        """
        if not runs:
            return runs

        def total_count(rs): return len(rs)
        m = total_count(runs)

        # Too many → merge adjacent small runs greedily
        if m > target_max:
            merged = []
            carry = None
            for r in runs:
                if carry is None:
                    carry = r
                    continue
                # merge carry with r
                carry = (carry[0], r[1])
                # Decide when to commit a merged run: keep merging until we reduce enough
                # Simple heuristic: commit whenever we would overshoot target if we skipped committing
                need = m - target_max
                # if we still need large reduction, keep merging; otherwise, commit
                if len(merged) + 1 + (1 if carry is not None else 0) >= target_min:
                    merged.append(carry)
                    carry = None
            if carry is not None:
                merged.append(carry)
            runs = [(s,e) for (s,e) in merged if e - s >= 2]

        # Too few → subdivide longest runs
        while len(runs) < target_min:
            # pick the longest run by length
            lens = [(e - s, i) for i,(s,e) in enumerate(runs)]
            L, i = max(lens)
            if L < 4:
                # can't split meaningfully; give up
                break
            s, e = runs.pop(i)
            mid = s + L // 2
            # ensure each side has >=2 points
            if mid - s < 2: mid = s + 2
            if e - mid < 2: mid = e - 2
            runs.insert(i,   (s, mid))
            runs.insert(i+1, (mid, e))

        return runs

    def _update_robot_xy(self, name: str):
        t, x, y = self.arr[name]
        segs = self.segments[name]

        if t.size < 2:
            for it in segs: it.setData([], [])
            self.start_markers[name].setData([], []); self.end_markers[name].setData([], [])
            return

        lo, hi = self.windows[name]
        i0 = int(np.searchsorted(t, DTYPE(lo), side='left'))
        i1 = int(np.searchsorted(t, DTYPE(hi), side='right'))
        if i1 - i0 < 2:
            for it in segs: it.setData([], [])
            self.start_markers[name].setData([], []); self.end_markers[name].setData([], [])
            return

        xs = x[i0:i1]; ys = y[i0:i1]

        # --- EITHER: split by gaps (your gap logic) ---
        dt = np.diff(t[i0:i1])
        dx = np.diff(xs); dy = np.diff(ys)
        breaks = (dt > DTYPE(MAX_DT_GAP_S))
        if MAX_DIST_GAP_M > 0.0:
            dd = np.sqrt(dx*dx + dy*dy)
            breaks |= (dd > DTYPE(MAX_DIST_GAP_M))

        idx = np.nonzero(breaks)[0] + 1
        starts = np.r_[0, idx]
        ends   = np.r_[idx-1, len(xs)-2]

        used = 0
        for k in range(len(starts)):
            s_edge = int(starts[k]); e_edge = int(ends[k])
            if e_edge < s_edge: 
                continue
            s_pt = s_edge
            e_pt = e_edge + 1
            if e_pt - s_pt + 1 < 2:
                continue
            if used >= len(segs):
                break
            segs[used].setData(xs[s_pt:e_pt+1], ys[s_pt:e_pt+1])
            used += 1

        # clear the rest
        for k in range(used, len(segs)):
            segs[k].setData([], [])

        # <<< THIS IS THE IMPORTANT PART: set alpha ramp for the used subset >>>
        self._set_alpha_ramp_for_used(name, used)

        # endpoints for the whole window
        self.start_markers[name].setData([float(xs[0])],  [float(ys[0])])
        self.end_markers[name].setData(  [float(xs[-1])], [float(ys[-1])])

    def _update_all_xy(self):
        for name in self.arr.keys():
            self._update_robot_xy(name)


def color_cycle(n: int) -> List[Tuple[int,int,int]]:
    base = [(87,227,137),
            (192,97,203),
            (255,163,72),
            (98,160,234),
            (255,99,132),
            (255,206,86),
            (75,192,192),
            (153,102,255)]
    out=[]; i=0
    while len(out) < n: out.append(base[i % len(base)]); i+=1
    return out


def main():
    DATA_ROOT = os.getenv("CU_MULTI_ROOT")
    if DATA_ROOT is None:
        sys.exit("ERROR: CU_MULTI_ROOT is not set.")

    env = "main_campus"  # or "kittredge_loop"
    robotIDs = [1, 2, 3, 4]

    robots: Dict[str, List[Pose]] = {}
    colors: Dict[str, Tuple[int,int,int]] = {}

    for robotID in robotIDs:
        name = f"{env}_r{robotID}"
        p = os.path.join(DATA_ROOT, "gt_poses", f"{env}_robot{robotID}_ref.csv")
        pts = load_csv(p)
        robots[name] = pts

    for name, col in zip(robots.keys(), color_cycle(len(robots))): colors[name] = col

    app = QtWidgets.QApplication(sys.argv)
    app.setStyleSheet(
        "QWidget { background-color: white; } QCheckBox, QLabel { background-color: white; }"
    )
    win = MultiTrajectoryClipper(robots, colors); win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
