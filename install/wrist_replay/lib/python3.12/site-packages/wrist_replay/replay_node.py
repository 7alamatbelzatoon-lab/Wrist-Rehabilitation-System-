#!/usr/bin/env python3
import csv
import math
from pathlib import Path
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def as_bool(v):
    if isinstance(v, bool): return v
    if isinstance(v, (int, float)): return bool(v)
    if isinstance(v, str): return v.strip().lower() in ('1','true','yes','on')
    return False


def parse_time(s):
    s = s.strip()
    for fmt in ("%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d %H:%M:%S"):
        try:
            return datetime.strptime(s, fmt)
        except ValueError:
            pass
    raise ValueError(f"Unsupported time format: {s!r}")


class CSVReplay(Node):
    """
    Smooth visual replay (interpolation) while labels/RED rule use RAW CSV.
    Also renders a tiny on-screen plot of Target vs Current (time window).
    """

    def __init__(self):
        super().__init__('replay_node')

        # -------- Params (same as before) --------
        self.declare_parameter('csv_path', str(Path.home() / 'ros2_ws/adaptive_log.csv'))
        self.declare_parameter('loop', False)
        self.declare_parameter('interp_rate', 60.0)
        self.declare_parameter('ease', True)
        self.declare_parameter('pos_err_thresh_deg', 5.0)

        self.csv_path = Path(self.get_parameter('csv_path').value)
        self.loop     = as_bool(self.get_parameter('loop').value)
        self.rate     = float(self.get_parameter('interp_rate').value)
        self.ease     = as_bool(self.get_parameter('ease').value)
        self.err_thr  = float(self.get_parameter('pos_err_thresh_deg').value)

        if not self.csv_path.exists():
            self.get_logger().error(f'CSV not found: {self.csv_path}')
            raise SystemExit(1)

        # -------- Load CSV (unchanged) --------
        raw_rows = self._load_csv(self.csv_path)
        if not raw_rows:
            self.get_logger().error('CSV has no usable rows')
            raise SystemExit(1)

        raw_rows.sort(key=lambda r: r['time'])
        # Distribute rows evenly within each same second so none get skipped.
        self._assign_rel_times_within_seconds(raw_rows)

        # RAW rows (with rel_time) drive labels & color; also used for plot
        self.raw_rows = raw_rows
        # Interp rows = same (now strictly increasing)
        self.rows = self.raw_rows

        self.total_dur = self.rows[-1]['rel_time']
        if self.total_dur <= 0:
            self.get_logger().error('CSV duration is zero — need ≥2 distinct timestamps.')
            raise SystemExit(1)

        # -------- Publishers --------
        self.pub_rad  = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_deg  = self.create_publisher(JointState, 'joint_states_deg', 10)
        qos = QoSProfile(depth=10,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_raw_deg = self.create_publisher(JointState, 'joint_states_raw_deg', qos)
        self.pub_marker  = self.create_publisher(Marker, 'replay_markers', 10)

        # -------- State --------
        self.idx = 0
        self.raw_idx = 0
        self.sim_start = self.now_sec()

        # Last RAW numbers for labels (init)
        self.last_raw_target_deg  = self.raw_rows[0]['target']
        self.last_raw_current_deg = self.raw_rows[0]['current']
        self.last_raw_index_1b    = 1
        self.last_raw_time_str    = self.raw_rows[0]['time'].strftime("%H:%M:%S")

        # --- tiny plot config (tweak if you want) ---
        self.plot_enabled = True
        self.plot_window_s = 10.0       # seconds visible in the plot
        self.plot_y_range = 40.0        # degrees shown as +/- range
        self.plot_w = 0.70              # meters (in RViz world)
        self.plot_h = 0.35
        # Position in base_link frame (XY plane). Move if it overlaps your robot:
        self.plot_origin = (0.0, 0.55, 0.01)  # x, y, z

        self.get_logger().info(
            f"Replay: {len(self.rows)} samples, duration={self.total_dur:.3f}s, "
            f"rate={self.rate:.1f} Hz, loop={self.loop}"
        )

        self.timer = self.create_timer(max(1.0/self.rate, 0.005), self._tick)

    # ---------- Helpers ----------
    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _load_csv(self, path: Path):
        rows = []
        with path.open('r', newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    t = parse_time(row['Time'])
                    target = float(row['Target'])
                    current = float(row['Current'])
                    rows.append({'time': t, 'target': target, 'current': current})
                except Exception:
                    continue
        return rows

    def _assign_rel_times_within_seconds(self, rows):
        t0 = rows[0]['time']

        # bucket rows by whole second offset from t0
        buckets = {}
        for r in rows:
            sec = int((r['time'] - t0).total_seconds())
            buckets.setdefault(sec, []).append(r)

        # spread evenly within each bucket
        for sec, bucket in sorted(buckets.items()):
            m = len(bucket)
            if m == 1:
                bucket[0]['rel_time'] = float(sec)
            else:
                step = 1.0 / m
                for i, r in enumerate(bucket):
                    r['rel_time'] = float(sec) + i * step
            # pretty time string
            for r in bucket:
                r['time_str'] = r['time'].strftime("%H:%M:%S")

        # ensure strict monotonicity (safety)
        prev = -1e9
        for r in rows:
            if r['rel_time'] <= prev:
                r['rel_time'] = prev + 1e-6
            prev = r['rel_time']

    def _smoothstep(self, a):  # optional easing
        return a*a*(3.0 - 2.0*a)

    def _lerp(self, a, y1, y2):
        return (1.0 - a) * y1 + a * y2

    # ---------- Main loop ----------
    def _tick(self):
        now = self.now_sec()
        sim_t = now - self.sim_start

        # wrap or finish
        if sim_t >= self.total_dur:
            if self.loop:
                wraps = int(sim_t // self.total_dur)
                self.sim_start += wraps * self.total_dur
                sim_t -= wraps * self.total_dur
                self.idx = 0
                self.raw_idx = 0
            else:
                last = self.rows[-1]
                self._publish_smooth(last['target'], last['current'])
                self._publish_labels(self.last_raw_target_deg, self.last_raw_current_deg)
                if self.plot_enabled:
                    self._publish_plot(sim_t)
                self.get_logger().info("Replay finished.")
                rclpy.shutdown()
                return

        # ---- Smooth interpolation (motion) ----
        while self.idx + 1 < len(self.rows) and self.rows[self.idx + 1]['rel_time'] <= sim_t:
            self.idx += 1
        r1 = self.rows[self.idx]
        r2 = self.rows[min(self.idx + 1, len(self.rows) - 1)]
        t1, t2 = r1['rel_time'], r2['rel_time']
        dt = max(t2 - t1, 1e-6)
        a = min(max((sim_t - t1) / dt, 0.0), 1.0)
        if self.ease:
            a = self._smoothstep(a)

        target_deg_i  = self._lerp(a, r1['target'],  r2['target'])
        current_deg_i = self._lerp(a, r1['current'], r2['current'])
        self._publish_smooth(target_deg_i, current_deg_i)

        # ---- RAW updates (labels & raw topic at CSV rate) ----
        while self.raw_idx < len(self.raw_rows) and self.raw_rows[self.raw_idx]['rel_time'] <= sim_t:
            rr = self.raw_rows[self.raw_idx]
            self.last_raw_target_deg  = rr['target']
            self.last_raw_current_deg = rr['current']
            self.last_raw_index_1b    = self.raw_idx + 1
            self.last_raw_time_str    = rr['time_str']
            self._publish_raw_deg(rr['target'], rr['current'])
            self.raw_idx += 1

        self._publish_labels(self.last_raw_target_deg, self.last_raw_current_deg)

        # ---- Plot (optional) ----
        if self.plot_enabled:
            self._publish_plot(sim_t)

    # ---------- Publishers ----------
    def _publish_smooth(self, target_deg, current_deg):
        stamp = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = stamp
        js.name = ['wrist_joint_doc', 'wrist_joint_pat']
        js.position = [math.radians(target_deg), math.radians(current_deg)]
        self.pub_rad.publish(js)

        jd = JointState()
        jd.header.stamp = stamp
        jd.name = js.name[:]
        jd.position = [target_deg, current_deg]
        self.pub_deg.publish(jd)

    def _publish_raw_deg(self, target_deg, current_deg):
        jd = JointState()
        jd.header.stamp = self.get_clock().now().to_msg()
        jd.name = ['wrist_joint_doc', 'wrist_joint_pat']
        jd.position = [target_deg, current_deg]
        self.pub_raw_deg.publish(jd)

    # ---- cleaner labels (attached to hands) ----
    def _publish_labels(self, target_deg_raw, current_deg_raw):
        # Doctor (white)
        m1 = Marker()
        m1.header.frame_id = 'hand_doc'
        m1.header.stamp = self.get_clock().now().to_msg()
        m1.ns = 'replay_labels'
        m1.id = 1
        m1.type = Marker.TEXT_VIEW_FACING
        m1.action = Marker.ADD
        m1.pose.position.z = 0.12
        m1.scale.z = 0.11
        m1.color.r, m1.color.g, m1.color.b, m1.color.a = (1.0, 1.0, 1.0, 1.0)
        m1.text = (
            f"Doctor\n"
            f"T: {target_deg_raw:>5.1f}°\n"
            f"Row: {self.last_raw_index_1b}\n"
            f"Time: {self.last_raw_time_str}"
        )
        self.pub_marker.publish(m1)

        # Patient (green/red)
        delta = current_deg_raw - target_deg_raw
        red = abs(delta) >= self.err_thr

        m2 = Marker()
        m2.header.frame_id = 'hand_pat'
        m2.header.stamp = m1.header.stamp
        m2.ns = 'replay_labels'
        m2.id = 2
        m2.type = Marker.TEXT_VIEW_FACING
        m2.action = Marker.ADD
        m2.pose.position.z = 0.12
        m2.scale.z = 0.11
        if red:
            m2.color.r, m2.color.g, m2.color.b, m2.color.a = (1.0, 0.25, 0.25, 1.0)
        else:
            m2.color.r, m2.color.g, m2.color.b, m2.color.a = (0.25, 1.0, 0.25, 1.0)
        m2.text = (
            f"Patient\n"
            f"C: {current_deg_raw:>5.1f}°\n"
            f"Δ: {delta:+5.1f}°"
        )
        self.pub_marker.publish(m2)

    # ---- tiny time-series plot in RViz (base_link frame) ----
    def _publish_plot(self, sim_t):
        x0, y0, z0 = self.plot_origin
        w, h = self.plot_w, self.plot_h
        ymin, ymax = -self.plot_y_range, self.plot_y_range

        # Collect RAW points in window
        t_start = sim_t - self.plot_window_s
        pts_t, pts_c = [], []
        for r in self.raw_rows:
            t = r['rel_time']
            if t < t_start: continue
            if t > sim_t:   break
            x = x0 + (t - t_start) / self.plot_window_s * w
            # map degrees to y
            yT = y0 + (r['target']  - ymin) / (ymax - ymin) * h
            yC = y0 + (r['current'] - ymin) / (ymax - ymin) * h
            pts_t.append(Point(x=x, y=yT, z=z0))
            pts_c.append(Point(x=x, y=yC, z=z0))

        # Border
        border = Marker()
        border.header.frame_id = 'base_link'
        border.header.stamp = self.get_clock().now().to_msg()
        border.ns = 'replay_plot'
        border.id = 10
        border.type = Marker.LINE_LIST
        border.action = Marker.ADD
        border.scale.x = 0.004
        border.color.r, border.color.g, border.color.b, border.color.a = (0.8, 0.8, 0.8, 1.0)
        bl = Point(x=x0,     y=y0,     z=z0)
        br = Point(x=x0 + w, y=y0,     z=z0)
        tl = Point(x=x0,     y=y0 + h, z=z0)
        tr = Point(x=x0 + w, y=y0 + h, z=z0)
        border.points = [bl, br, br, tr, tr, tl, tl, bl]
        self.pub_marker.publish(border)

        # Zero line (y=0)
        zero_y = y0 + (0 - ymin) / (ymax - ymin) * h
        zero = Marker()
        zero.header.frame_id = 'base_link'
        zero.header.stamp = border.header.stamp
        zero.ns = 'replay_plot'
        zero.id = 11
        zero.type = Marker.LINE_LIST
        zero.action = Marker.ADD
        zero.scale.x = 0.003
        zero.color.r, zero.color.g, zero.color.b, zero.color.a = (0.6, 0.6, 0.6, 1.0)
        zero.points = [Point(x=x0, y=zero_y, z=z0), Point(x=x0 + w, y=zero_y, z=z0)]
        self.pub_marker.publish(zero)

        # Target curve (white)
        line_t = Marker()
        line_t.header.frame_id = 'base_link'
        line_t.header.stamp = border.header.stamp
        line_t.ns = 'replay_plot'
        line_t.id = 20
        line_t.type = Marker.LINE_STRIP
        line_t.action = Marker.ADD
        line_t.scale.x = 0.006
        line_t.color.r, line_t.color.g, line_t.color.b, line_t.color.a = (1.0, 1.0, 1.0, 1.0)
        line_t.points = pts_t
        self.pub_marker.publish(line_t)

        # Current curve (green)
        line_c = Marker()
        line_c.header.frame_id = 'base_link'
        line_c.header.stamp = border.header.stamp
        line_c.ns = 'replay_plot'
        line_c.id = 21
        line_c.type = Marker.LINE_STRIP
        line_c.action = Marker.ADD
        line_c.scale.x = 0.006
        line_c.color.r, line_c.color.g, line_c.color.b, line_c.color.a = (0.2, 1.0, 0.2, 1.0)
        line_c.points = pts_c
        self.pub_marker.publish(line_c)


def main():
    rclpy.init()
    node = CSVReplay()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
