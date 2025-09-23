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
NUMPY_DATA_TYPE = np.float64
MAX_PREALLOCATED_SEGMENTS_PER_ROBOT = 128    # preallocated faded segments per robot
TRAJECTORY_LINE_WIDTH = 8     # thicker lines for visibility
MAX_TIME_GAP_SECONDS = 1    # split if time gap between neighbors > this
MAX_DISTANCE_GAP_METERS = 0.5   # split if spatial jump > this (0.0 => disable distance check)
MIN_ALPHA_VALUE = 100
MAX_ALPHA_VALUE = 255

MAX_SNAP_TIME_DIFFERENCE_SECONDS = 0.5   # only snap if a sample is within 0.5 s (tweak)

@dataclass
class Pose:
    t: float
    x: float
    y: float


# ---------- csv loader ----------
def load_csv(csv_file_path: str) -> List[Pose]:
    pose_list: List[Pose] = []
    with open(csv_file_path, "r", newline="") as csv_file:
        next(csv_file)  # Skip first header line
        next(csv_file)  # Skip second header line
        csv_reader = csv.DictReader(csv_file)
        if not {"# timestamp", " x", " y"}.issubset(csv_reader.fieldnames or set()):
            raise ValueError("CSV must have headers: '# timestamp',' x',' y'")
        for csv_row in csv_reader:
            try:
                pose_list.append(Pose(float(csv_row["# timestamp"]), float(csv_row[" x"]), float(csv_row[" y"])))
            except Exception:
                continue
    pose_list.sort(key=lambda pose: pose.t)
    return pose_list


def linear_curve(time_array: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Simple 0..1 curve over time_array (assumes time_array starts at 0)."""
    if time_array.size < 2:
        default_time_array = np.array([0.0, 1.0], dtype=NUMPY_DATA_TYPE)
        return default_time_array, default_time_array.copy()
    if time_array[-1] <= 0:
        return time_array, np.zeros_like(time_array)
    normalized_values = time_array / max(1e-9, time_array[-1])
    return time_array, normalized_values.astype(NUMPY_DATA_TYPE, copy=False)


class MultiTrajectoryClipper(QtWidgets.QMainWindow):
    def __init__(self, robot_trajectories: Dict[str, List[Pose]], robot_colors: Dict[str, Tuple[int,int,int]]):
        super().__init__()
        self.setWindowTitle("CU-Multi Trajectory Clipper")
        self.resize(1300, 900)

        self.robot_colors = robot_colors

        # shift to t0 = 0 per robot, but keep absolute t0 for export
        self.robot_trajectory_data: Dict[str, Tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
        self.robot_durations_seconds: Dict[str, float] = {}
        self.robot_absolute_start_times: Dict[str, float] = {}  # absolute first timestamp per robot
        for robot_name, pose_list in robot_trajectories.items():
            absolute_timestamps = np.asarray([pose.t for pose in pose_list], dtype=NUMPY_DATA_TYPE)
            if absolute_timestamps.size:
                self.robot_absolute_start_times[robot_name] = float(absolute_timestamps[0])
                relative_timestamps = absolute_timestamps - absolute_timestamps[0]
            else:
                self.robot_absolute_start_times[robot_name] = 0.0
                relative_timestamps = absolute_timestamps
            x_coordinates = np.asarray([pose.x for pose in pose_list], dtype=NUMPY_DATA_TYPE)
            y_coordinates = np.asarray([pose.y for pose in pose_list], dtype=NUMPY_DATA_TYPE)
            self.robot_trajectory_data[robot_name] = (relative_timestamps, x_coordinates, y_coordinates)
            self.robot_durations_seconds[robot_name] = float(relative_timestamps[-1]) if relative_timestamps.size else 0.0

        self.global_maximum_duration = max(self.robot_durations_seconds.values()) if self.robot_durations_seconds else 1.0

        # per-robot windows (zero-shifted seconds)
        self.robot_clip_windows: Dict[str, Tuple[float, float]] = {}
        for robot_name, duration_seconds in self.robot_durations_seconds.items():
            if duration_seconds > 0:
                self.robot_clip_windows[robot_name] = (0.10 * duration_seconds, 0.60 * duration_seconds)
            else:
                self.robot_clip_windows[robot_name] = (0.0, 0.0)

        # ---- UI ----
        central_widget = QtWidgets.QWidget(); self.setCentralWidget(central_widget)
        main_vertical_layout = QtWidgets.QVBoxLayout(central_widget)

        main_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical); main_splitter.setChildrenCollapsible(False)
        main_vertical_layout.addWidget(main_splitter)

        # XY plot (just trajectories)
        pg.setConfigOptions(antialias=True)
        self.trajectory_xy_plot = pg.PlotWidget()
        self.trajectory_xy_plot.setBackground("w")
        self.trajectory_xy_plot.hideAxis("bottom"); self.trajectory_xy_plot.hideAxis("left")
        self.trajectory_xy_plot.showGrid(x=False, y=False)
        self.trajectory_xy_plot.setAspectLocked(True, ratio=1)
        main_splitter.addWidget(self.trajectory_xy_plot)

        # bottom controls + single timeline
        bottom_control_widget = QtWidgets.QWidget(); bottom_vertical_layout = QtWidgets.QVBoxLayout(bottom_control_widget)
        bottom_vertical_layout.setContentsMargins(6, 6, 6, 6)
        main_splitter.addWidget(bottom_control_widget)

        control_horizontal_layout = QtWidgets.QHBoxLayout()
        control_horizontal_layout.addWidget(QtWidgets.QLabel("Robot:"))
        self.robot_selection_combo_box = QtWidgets.QComboBox()
        for robot_name in self.robot_trajectory_data.keys(): self.robot_selection_combo_box.addItem(robot_name)
        control_horizontal_layout.addWidget(self.robot_selection_combo_box)

        self.report_clip_range_button = QtWidgets.QPushButton("Report Clip Range")
        # self.report_all_btn = QtWidgets.QPushButton("Report Robot")
        self.export_ros2_bags_button = QtWidgets.QPushButton("Retime ROS2 bags")
        # control_horizontal_layout.addWidget(self.report_btn)
        # control_horizontal_layout.addWidget(self.report_all_btn)
        control_horizontal_layout.addWidget(self.export_ros2_bags_button)
        control_horizontal_layout.addStretch(1)
        bottom_vertical_layout.addLayout(control_horizontal_layout)

        # Single timeline
        self.timeline_plot = pg.PlotWidget()
        self.timeline_plot.setBackground("w")
        self.timeline_plot.hideAxis("bottom")
        self.timeline_plot.hideAxis("left")
        self.timeline_plot.setMouseEnabled(x=False, y=False)
        bottom_vertical_layout.addWidget(self.timeline_plot, stretch=1)

        # Background timelines & faint windows
        self.background_timeline_curves: Dict[str, pg.PlotDataItem] = {}
        self.background_clip_regions: Dict[str, pg.LinearRegionItem] = {}
        for robot_name in self.robot_trajectory_data.keys():
            time_array, _, _ = self.robot_trajectory_data[robot_name]
            timeline_x_values, timeline_y_values = linear_curve(time_array if time_array.size else np.array([0,1], dtype=NUMPY_DATA_TYPE))
            robot_color = self.robot_colors[robot_name]
            background_curve = self.timeline_plot.plot(timeline_x_values, timeline_y_values, pen=pg.mkPen(robot_color[0], robot_color[1], robot_color[2], 70, width=1.0))
            background_curve.setZValue(-20)
            self.background_timeline_curves[robot_name] = background_curve

            window_start, window_end = self.robot_clip_windows[robot_name]
            background_region = pg.LinearRegionItem(values=(window_start, window_end), bounds=[0.0, max(1.0, self.robot_durations_seconds[robot_name])], movable=False)
            background_region.setBrush(pg.mkBrush(robot_color[0], robot_color[1], robot_color[2], 28))
            for region_line in background_region.lines:
                region_line.setPen(pg.mkPen(robot_color[0], robot_color[1], robot_color[2], 80, width=1))
            background_region.setZValue(-15)
            self.timeline_plot.addItem(background_region)
            self.background_clip_regions[robot_name] = background_region

        # global black baseline time curve
        global_time_array = np.linspace(0.0, self.global_maximum_duration if self.global_maximum_duration > 0 else 1.0, num=200, dtype=NUMPY_DATA_TYPE)
        global_timeline_x, global_timeline_y = linear_curve(global_time_array)
        self.global_baseline_curve = self.timeline_plot.plot(global_timeline_x, global_timeline_y, pen=pg.mkPen(0, 0, 0, 130, width=1.0))
        self.global_baseline_curve.setZValue(-10)

        # Selected robot curve (colored)
        self.selected_robot_curve = self.timeline_plot.plot([], [], pen=pg.mkPen(0, 0, 0, 180, width=1.8))
        self.selected_robot_curve.setZValue(-5)

        # Editable region (selected robot only)
        self.editable_clip_region = pg.LinearRegionItem(values=(0.0, 0.0), bounds=[0.0, max(1.0, self.global_maximum_duration)], movable=True)
        self.timeline_plot.addItem(self.editable_clip_region)

        # Lock ranges
        timeline_view_box = self.timeline_plot.getViewBox()
        timeline_view_box.setLimits(xMin=0.0, xMax=self.global_maximum_duration, minXRange=self.global_maximum_duration, maxXRange=self.global_maximum_duration)
        self.timeline_plot.setXRange(0.0, self.global_maximum_duration, padding=0.0)
        self.timeline_plot.setYRange(0.0, 1.0, padding=0.0)

        # XY preallocated items
        self.trajectory_segments: Dict[str, List[pg.PlotDataItem]] = {}
        self.trajectory_start_markers: Dict[str, pg.ScatterPlotItem] = {}
        self.trajectory_end_markers: Dict[str, pg.ScatterPlotItem] = {}
        for robot_name, (time_array, x_coordinates, y_coordinates) in self.robot_trajectory_data.items():
            robot_color = self.robot_colors[robot_name]
            trajectory_segments: List[pg.PlotDataItem] = []
            for segment_index in range(MAX_PREALLOCATED_SEGMENTS_PER_ROBOT):
                alpha_value = int(MIN_ALPHA_VALUE + 190 * (segment_index / max(1, MAX_PREALLOCATED_SEGMENTS_PER_ROBOT - 1)))  # 40..230 across time
                segment_pen = pg.mkPen(robot_color[0], robot_color[1], robot_color[2], alpha_value, width=TRAJECTORY_LINE_WIDTH)
                segment_item = self.trajectory_xy_plot.plot([], [], pen=segment_pen, clear=False)
                segment_item.setClipToView(True)
                segment_item.setDownsampling(auto=False, method='peak')
                trajectory_segments.append(segment_item)
            self.trajectory_segments[robot_name] = trajectory_segments
            start_marker = pg.ScatterPlotItem(size=9, brush=pg.mkBrush(robot_color[0], robot_color[1], robot_color[2], 155), pen=pg.mkPen(None))
            end_marker   = pg.ScatterPlotItem(size=11, brush=pg.mkBrush(robot_color[0], robot_color[1], robot_color[2], 255), pen=pg.mkPen(30,30,30,30))
            self.trajectory_xy_plot.addItem(start_marker); self.trajectory_xy_plot.addItem(end_marker)
            self.trajectory_start_markers[robot_name] = start_marker; self.trajectory_end_markers[robot_name] = end_marker

        # Events
        self._is_switching_robots = False
        self.robot_selection_combo_box.currentTextChanged.connect(self._on_robot_changed)
        self.editable_clip_region.sigRegionChanged.connect(self._on_region_changed)
        if hasattr(self.editable_clip_region, "sigRegionChangeFinished"):
            self.editable_clip_region.sigRegionChangeFinished.connect(self._on_region_change_finished)

        self.report_clip_range_button.clicked.connect(self._on_report)
        # self.report_all_btn.clicked.connect(self._on_report_all)
        self.export_ros2_bags_button.clicked.connect(self._on_clip_ros2_bags)

        # Initial render
        first_robot_name = self.robot_selection_combo_box.currentText() if self.robot_selection_combo_box.count() else None
        self._on_robot_changed(first_robot_name)
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

    def _snap_to_sample(self, robot_name: str, time_value: float) -> float:
        """Snap to nearest sample only if it's close and not across a big gap."""
        time_array, _, _ = self.robot_trajectory_data[robot_name]
        if time_array.size == 0:
            return 0.0

        search_index = int(np.searchsorted(time_array, time_value))
        search_index = max(0, min(search_index, time_array.size - 1))

        # pick nearest of (search_index-1, search_index)
        candidate_indices = [search_index]
        if search_index > 0: candidate_indices.append(search_index - 1)
        # compute distances
        time_difference_candidates = [(abs(time_value - float(time_array[index])), index) for index in candidate_indices]
        minimum_time_difference, best_index = min(time_difference_candidates, key=lambda candidate: candidate[0])

        # respect large time gaps (MAX_TIME_GAP_SECONDS). If we chose best_index next to a big gap and
        # the other side is closer in time, prefer the closer one—but only if within MAX_SNAP_TIME_DIFFERENCE_SECONDS.
        if minimum_time_difference > MAX_SNAP_TIME_DIFFERENCE_SECONDS:
            # don't snap if nothing is close enough
            return time_value

        return float(time_array[best_index])


    def _idx_from_time(self, time_array: np.ndarray, time_value: float) -> int:
        search_index = int(np.searchsorted(time_array, NUMPY_DATA_TYPE(time_value)))
        return max(0, min(search_index, time_array.size-1))

    # ---- Reporting ----
    def _on_report(self):
        if not self.robot_clip_windows:
            QtWidgets.QMessageBox.information(self, "Clip Range", "No windows set.")
            return
        window_start_times = [window[0] for window in self.robot_clip_windows.values()]
        window_end_times = [window[1] for window in self.robot_clip_windows.values()]
        earliest_start_time = float(min(window_start_times)) if window_start_times else 0.0
        latest_end_time   = float(max(window_end_times)) if window_end_times else 0.0
        report_message = f"Earliest start (zero-shifted): {earliest_start_time:.3f} s\nLatest end (zero-shifted): {latest_end_time:.3f} s"
        print(report_message)
        QtWidgets.QMessageBox.information(self, "Clip Range", report_message)

    def _on_report_all(self):
        report_lines = []
        report_lines.append("Per-Robot Clip Windows (zero-shifted seconds)")
        report_lines.append("Robot, start_s, end_s, span_s, duration_s, trim_before_s, trim_after_s")
        for robot_name, (window_start, window_end) in self.robot_clip_windows.items():
            robot_duration = self.robot_durations_seconds.get(robot_name, 0.0)
            window_span = max(0.0, window_end - window_start)
            trim_before_seconds = max(0.0, window_start)
            trim_after_seconds  = max(0.0, robot_duration - window_end)
            report_lines.append(f"{robot_name}, {window_start:.3f}, {window_end:.3f}, {window_span:.3f}, {robot_duration:.3f}, {trim_before_seconds:.3f}, {trim_after_seconds:.3f}")
        report_text = "\n".join(report_lines)
        print(report_text)
        QtWidgets.QMessageBox.information(self, "Per-Robot Windows", report_text)

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

        robot_time_windows: Dict[str, DecimalPair] = {}
        # earliest = None
        # latest = None

        for robot_name in self.robot_trajectory_data.keys():
            clip_window = self.robot_clip_windows.get(robot_name)

            absolute_start_time = self.robot_absolute_start_times.get(robot_name)
            unix_start_time = float(absolute_start_time + clip_window[0])
            unix_end_time = float(absolute_start_time + clip_window[1])

            start_time_decimal = Decimal(unix_start_time)
            end_time_decimal = Decimal(unix_end_time)
            robot_time_windows[robot_name] = (start_time_decimal, end_time_decimal)
        
            # if earliest is None or unix_start_time < earliest: 
            #     earliest = unix_start_time
            # if latest   is None or unix_end_time > latest:   
            #     latest   = unix_end_time
            print(f"robot_name: {robot_name}, unix_start_time: {unix_start_time}")

        clip_ros2_bags.clip_robot_bags(robot_time_windows)


    def _on_export_unix_csv(self):
        """Export CSV with absolute UNIX times:
           unix_start_ts = robot_absolute_start_times[robot_name] + window_start
           unix_end_ts   = robot_absolute_start_times[robot_name] + window_end
        """
        if not self.robot_clip_windows:
            QtWidgets.QMessageBox.information(self, "Export", "No windows set.")
            return
        file_path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save UNIX Windows CSV",
                                                        "robot_windows_unix.csv",
                                                        "CSV files (*.csv)")
        if not file_path:
            return
        earliest_unix_time = None; latest_unix_time = None
        with open(file_path, "w", newline="") as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(["robot", "unix_start_ts", "unix_end_ts"])
            for robot_name in self.robot_trajectory_data.keys():
                window_start, window_end = self.robot_clip_windows.get(robot_name, (0.0, 0.0))
                absolute_start_time = self.robot_absolute_start_times.get(robot_name, 0.0)
                absolute_start_unix = float(absolute_start_time + window_start)
                absolute_end_unix = float(absolute_start_time + window_end)

                if earliest_unix_time is None or absolute_start_unix < earliest_unix_time: 
                    earliest_unix_time = absolute_start_unix
                if latest_unix_time   is None or absolute_end_unix > latest_unix_time:   
                    latest_unix_time   = absolute_end_unix

                csv_writer.writerow([robot_name, f"{absolute_start_unix:.9f}", f"{absolute_end_unix:.9f}"])
        export_message = f"Saved UNIX windows CSV:\n{file_path}\nGlobal range: [{earliest_unix_time:.9f}, {latest_unix_time:.9f}]"
        print(export_message)
        QtWidgets.QMessageBox.information(self, "Export", export_message)

    # ---- selection / region ----
    def _on_robot_changed(self, robot_name: str | None):
        if not robot_name:
            return
        self._is_switching_robots = True
        try:
            # Selected colored curve
            time_array, _, _ = self.robot_trajectory_data[robot_name]
            timeline_x_values, timeline_y_values = linear_curve(time_array if time_array.size else np.array([0,1], dtype=NUMPY_DATA_TYPE))
            self.selected_robot_curve.setData(timeline_x_values, timeline_y_values)
            robot_color = self.robot_colors[robot_name]
            self.selected_robot_curve.setPen(pg.mkPen(*robot_color, 220, width=10))

            # Update editable region bounds and value
            robot_duration = self.robot_durations_seconds[robot_name]
            self.editable_clip_region.setBounds([0.0, max(1e-9, robot_duration)])
            window_start, window_end = self.robot_clip_windows[robot_name]
            window_start = max(0.0, min(window_start, robot_duration)); window_end = max(window_start, min(window_end, robot_duration)) if robot_duration > 0 else 0.0
            self.editable_clip_region.blockSignals(True); self.editable_clip_region.setRegion((window_start, window_end)); self.editable_clip_region.blockSignals(False)

            # Recolor region
            region_fill_brush = pg.mkBrush(robot_color[0], robot_color[1], robot_color[2], 125)
            region_edge_pen = pg.mkPen(robot_color, width=10)
            self.editable_clip_region.setBrush(region_fill_brush); [line.setPen(region_edge_pen) for line in self.editable_clip_region.lines]
        finally:
            self._is_switching_robots = False

    def _on_region_changed(self):
        if self._is_switching_robots:
            return
        
        current_robot_name = self.robot_selection_combo_box.currentText()
        if not current_robot_name:
            return
        
        region_start, region_end = self.editable_clip_region.getRegion()
        self.robot_clip_windows[current_robot_name] = (float(region_start), float(region_end))
        background_region = self.background_clip_regions.get(current_robot_name)
        
        if background_region is not None:
            background_region.setRegion((float(region_start), float(region_end)))
        
        self._update_robot_xy(current_robot_name)

    def _on_region_change_finished(self):
        """Snap region edges to the nearest samples of the selected robot after drag."""
        current_robot_name = self.robot_selection_combo_box.currentText()

        if not current_robot_name:
            return
        
        region_start, region_end = self.editable_clip_region.getRegion()
        snapped_start = self._snap_to_sample(current_robot_name, region_start)
        snapped_end = self._snap_to_sample(current_robot_name, region_end)

        if snapped_end < snapped_start:
            snapped_end = snapped_start

        self.editable_clip_region.blockSignals(True)
        self.editable_clip_region.setRegion((snapped_start, snapped_end))
        self.editable_clip_region.blockSignals(False)

        self.robot_clip_windows[current_robot_name] = (snapped_start, snapped_end)
        background_region = self.background_clip_regions.get(current_robot_name)
        
        if background_region is not None:
            background_region.setRegion((snapped_start, snapped_end))
        
        self._update_robot_xy(current_robot_name)

    def _shift_window(self, sample_shift: int, is_large_shift: bool):
        current_robot_name = self.robot_selection_combo_box.currentText()
        if not current_robot_name: return
        time_array, _, _ = self.robot_trajectory_data[current_robot_name]
        window_start, window_end = self.robot_clip_windows[current_robot_name]
        start_index = self._idx_from_time(time_array, window_start)
        end_index = self._idx_from_time(time_array, window_end)
        start_index = max(0, min(start_index + sample_shift, time_array.size-1))
        end_index = max(start_index, min(end_index + sample_shift, time_array.size-1))
        new_start_time = float(time_array[start_index]); new_end_time = float(time_array[end_index])
        self.editable_clip_region.blockSignals(True); self.editable_clip_region.setRegion((new_start_time, new_end_time)); self.editable_clip_region.blockSignals(False)
        self.robot_clip_windows[current_robot_name] = (new_start_time, new_end_time)
        background_region = self.background_clip_regions.get(current_robot_name)
        if background_region is not None:
            background_region.setRegion((new_start_time, new_end_time))
        self._update_robot_xy(current_robot_name)


    def _set_alpha_ramp_for_used(self, robot_name: str, used_segment_count: int):
        """Re-ink pens for the first `used_segment_count` segments so alpha ramps from head->tail."""
        robot_color = self.robot_colors[robot_name]
        trajectory_segments = self.trajectory_segments[robot_name]
        if used_segment_count <= 0:
            return
        # ramp MIN_ALPHA_VALUE .. MAX_ALPHA_VALUE across the *actually used* segments
        for segment_index in range(used_segment_count):
            alpha_value = int(np.interp(segment_index, [0, max(1, used_segment_count - 1)], [MIN_ALPHA_VALUE, MAX_ALPHA_VALUE]))
            trajectory_segments[segment_index].setPen(pg.mkPen(robot_color[0], robot_color[1], robot_color[2], alpha_value, width=TRAJECTORY_LINE_WIDTH))

    def _normalize_runs(self, trajectory_runs, minimum_target_count, maximum_target_count):
        """
        trajectory_runs: list of (start_index, end_index) index pairs (exclusive end), each with length >= 2
        Returns a new list of runs whose count is clamped into [minimum_target_count, maximum_target_count]
        by merging or subdividing evenly (purely visual).
        """
        if not trajectory_runs:
            return trajectory_runs

        def get_total_count(run_list): return len(run_list)
        current_run_count = get_total_count(trajectory_runs)

        # Too many → merge adjacent small runs greedily
        if current_run_count > maximum_target_count:
            merged_runs = []
            carry_run = None
            for current_run in trajectory_runs:
                if carry_run is None:
                    carry_run = current_run
                    continue
                # merge carry_run with current_run
                carry_run = (carry_run[0], current_run[1])
                # Decide when to commit a merged run: keep merging until we reduce enough
                # Simple heuristic: commit whenever we would overshoot target if we skipped committing
                reduction_needed = current_run_count - maximum_target_count
                # if we still need large reduction, keep merging; otherwise, commit
                if len(merged_runs) + 1 + (1 if carry_run is not None else 0) >= minimum_target_count:
                    merged_runs.append(carry_run)
                    carry_run = None
            if carry_run is not None:
                merged_runs.append(carry_run)
            trajectory_runs = [(start_idx, end_idx) for (start_idx, end_idx) in merged_runs if end_idx - start_idx >= 2]

        # Too few → subdivide longest runs
        while len(trajectory_runs) < minimum_target_count:
            # pick the longest run by length
            run_lengths = [(end_idx - start_idx, run_index) for run_index, (start_idx, end_idx) in enumerate(trajectory_runs)]
            longest_length, longest_run_index = max(run_lengths)
            if longest_length < 4:
                # can't split meaningfully; give up
                break
            start_idx, end_idx = trajectory_runs.pop(longest_run_index)
            middle_index = start_idx + longest_length // 2
            # ensure each side has >=2 points
            if middle_index - start_idx < 2: middle_index = start_idx + 2
            if end_idx - middle_index < 2: middle_index = end_idx - 2
            trajectory_runs.insert(longest_run_index,   (start_idx, middle_index))
            trajectory_runs.insert(longest_run_index+1, (middle_index, end_idx))

        return trajectory_runs

    def _update_robot_xy(self, robot_name: str):
        time_array, x_coordinates, y_coordinates = self.robot_trajectory_data[robot_name]
        trajectory_segments = self.trajectory_segments[robot_name]

        if time_array.size < 2:
            for segment_item in trajectory_segments: segment_item.setData([], [])
            self.trajectory_start_markers[robot_name].setData([], []); self.trajectory_end_markers[robot_name].setData([], [])
            return

        window_start, window_end = self.robot_clip_windows[robot_name]
        start_index = int(np.searchsorted(time_array, NUMPY_DATA_TYPE(window_start), side='left'))
        end_index = int(np.searchsorted(time_array, NUMPY_DATA_TYPE(window_end), side='right'))
        if end_index - start_index < 2:
            for segment_item in trajectory_segments: segment_item.setData([], [])
            self.trajectory_start_markers[robot_name].setData([], []); self.trajectory_end_markers[robot_name].setData([], [])
            return

        window_x_coordinates = x_coordinates[start_index:end_index]; window_y_coordinates = y_coordinates[start_index:end_index]

        # --- EITHER: split by gaps (your gap logic) ---
        time_differences = np.diff(time_array[start_index:end_index])
        x_differences = np.diff(window_x_coordinates); y_differences = np.diff(window_y_coordinates)
        gap_breaks = (time_differences > NUMPY_DATA_TYPE(MAX_TIME_GAP_SECONDS))
        if MAX_DISTANCE_GAP_METERS > 0.0:
            distance_differences = np.sqrt(x_differences*x_differences + y_differences*y_differences)
            gap_breaks |= (distance_differences > NUMPY_DATA_TYPE(MAX_DISTANCE_GAP_METERS))

        break_indices = np.nonzero(gap_breaks)[0] + 1
        segment_start_indices = np.r_[0, break_indices]
        segment_end_indices   = np.r_[break_indices-1, len(window_x_coordinates)-2]

        used_segment_count = 0
        for segment_index in range(len(segment_start_indices)):
            segment_start_edge = int(segment_start_indices[segment_index]); segment_end_edge = int(segment_end_indices[segment_index])
            if segment_end_edge < segment_start_edge: 
                continue
            segment_start_point = segment_start_edge
            segment_end_point = segment_end_edge + 1
            if segment_end_point - segment_start_point + 1 < 2:
                continue
            if used_segment_count >= len(trajectory_segments):
                break
            trajectory_segments[used_segment_count].setData(window_x_coordinates[segment_start_point:segment_end_point+1], window_y_coordinates[segment_start_point:segment_end_point+1])
            used_segment_count += 1

        # clear the rest
        for segment_index in range(used_segment_count, len(trajectory_segments)):
            trajectory_segments[segment_index].setData([], [])

        # <<< THIS IS THE IMPORTANT PART: set alpha ramp for the used subset >>>
        self._set_alpha_ramp_for_used(robot_name, used_segment_count)

        # endpoints for the whole window
        self.trajectory_start_markers[robot_name].setData([float(window_x_coordinates[0])],  [float(window_y_coordinates[0])])
        self.trajectory_end_markers[robot_name].setData(  [float(window_x_coordinates[-1])], [float(window_y_coordinates[-1])])

    def _update_all_xy(self):
        for robot_name in self.robot_trajectory_data.keys():
            self._update_robot_xy(robot_name)


def generate_robot_color_cycle(robot_count: int) -> List[Tuple[int,int,int]]:
    base_color_palette = [(87,227,137),
            (192,97,203),
            (255,163,72),
            (98,160,234),
            (255,99,132),
            (255,206,86),
            (75,192,192),
            (153,102,255)]
    color_list=[]; color_index=0
    while len(color_list) < robot_count: color_list.append(base_color_palette[color_index % len(base_color_palette)]); color_index+=1
    return color_list


def main():
    data_root_directory = os.getenv("CU_MULTI_ROOT")
    if data_root_directory is None:
        sys.exit("ERROR: CU_MULTI_ROOT is not set.")

    environment_name = "main_campus"  # or "kittredge_loop"
    robot_id_list = [1, 2, 3, 4]

    robot_trajectories: Dict[str, List[Pose]] = {}
    robot_colors: Dict[str, Tuple[int,int,int]] = {}

    for robot_id in robot_id_list:
        robot_name = f"{environment_name}_r{robot_id}"
        csv_file_path = os.path.join(
            data_root_directory, 
            f"{environment_name}", 
            f"robot{robot_id}", f"robot{robot_id}_{environment_name}_gt_utm_poses.csv"
        )
        pose_list = load_csv(csv_file_path)
        robot_trajectories[robot_name] = pose_list

    for robot_name, robot_color in zip(robot_trajectories.keys(), generate_robot_color_cycle(len(robot_trajectories))): 
        robot_colors[robot_name] = robot_color

    qt_application = QtWidgets.QApplication(sys.argv)
    qt_application.setStyleSheet(
        "QWidget { background-color: white; } QCheckBox, QLabel { background-color: white; }"
    )
    trajectory_clipper_window = MultiTrajectoryClipper(robot_trajectories, robot_colors)
    trajectory_clipper_window.show()
    sys.exit(qt_application.exec())


if __name__ == "__main__":
    main()
