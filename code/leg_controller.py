"""Interactive USB serial controller and 3D visualizer for a three-servo leg."""

from __future__ import annotations

import argparse
import math
import queue
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

COXA_LENGTH = 74.97
FEMUR_LENGTH = 150.0
TIBIA_LENGTH = 284.4615

# Raw Servo.write() degree mapping. Calibrated servos use:
# 0 = left, 90 = center, 180 = right.
COXA_ZERO_DEG = 90.0
COXA_DIRECTION = -1.0
FEMUR_ZERO_DEG = 90.0
FEMUR_DIRECTION = -1.0
TIBIA_ZERO_DEG = 90.0
TIBIA_DIRECTION = -1.0

HOME = (450.0, 0.0, -30.0)
GAIT_FRONT = (450.0, 60.0, -30.0)
GAIT_REAR = (450.0, -60.0, -30.0)
APPROACH_MS = 450
STANCE_MS = 650
SWING_MS = 580
SWING_HEIGHT = 100.0
AXIS_LIMITS = ((80.0, 460.0), (-220.0, 220.0), (-250.0, 180.0))
DEFAULT_CAMERA_AZIMUTH_DEG = 35.0
DEFAULT_CAMERA_ELEVATION_DEG = 30.0


@dataclass
class LegSolution:
    hip: tuple[float, float, float]
    coxa: tuple[float, float, float]
    knee: tuple[float, float, float]
    foot: tuple[float, float, float]
    angles: tuple[float, float, float]


def solve_leg(position: tuple[float, float, float]) -> Optional[LegSolution]:
    """Return drawable joints and firmware-matching servo angles, or None."""
    x, y, z = position
    horizontal = math.hypot(x, y)
    planar = horizontal - COXA_LENGTH
    distance = math.hypot(planar, z)
    min_reach = abs(TIBIA_LENGTH - FEMUR_LENGTH)
    max_reach = TIBIA_LENGTH + FEMUR_LENGTH

    if horizontal <= COXA_LENGTH or distance < min_reach or distance > max_reach:
        return None

    femur_cos = (FEMUR_LENGTH**2 + distance**2 - TIBIA_LENGTH**2) / (
        2.0 * FEMUR_LENGTH * distance
    )
    knee_cos = (FEMUR_LENGTH**2 + TIBIA_LENGTH**2 - distance**2) / (
        2.0 * FEMUR_LENGTH * TIBIA_LENGTH
    )
    femur_cos = max(-1.0, min(1.0, femur_cos))
    knee_cos = max(-1.0, min(1.0, knee_cos))

    yaw = math.degrees(math.atan2(y, x))
    femur_angle = math.degrees(math.atan2(z, planar) + math.acos(femur_cos))
    knee_bend = 180.0 - math.degrees(math.acos(knee_cos))
    angles = (
        COXA_ZERO_DEG + COXA_DIRECTION * yaw,
        FEMUR_ZERO_DEG + FEMUR_DIRECTION * femur_angle,
        TIBIA_ZERO_DEG + TIBIA_DIRECTION * knee_bend,
    )
    if not all(0.0 <= angle <= 180.0 for angle in angles):
        return None

    radial_x = x / horizontal
    radial_y = y / horizontal
    coxa = (COXA_LENGTH * radial_x, COXA_LENGTH * radial_y, 0.0)

    along = (FEMUR_LENGTH**2 - TIBIA_LENGTH**2 + distance**2) / (2.0 * distance)
    height = math.sqrt(max(0.0, FEMUR_LENGTH**2 - along**2))
    local_knee_out = along * planar / distance - height * z / distance
    local_knee_z = along * z / distance + height * planar / distance
    knee_radius = COXA_LENGTH + local_knee_out
    knee = (knee_radius * radial_x, knee_radius * radial_y, local_knee_z)

    return LegSolution((0.0, 0.0, 0.0), coxa, knee, position, angles)


class SerialTransport:
    def __init__(self, port: Optional[str], baud: int, simulate: bool) -> None:
        self.responses: queue.Queue[str] = queue.Queue()
        self.connection = None
        self.stopped = threading.Event()
        self.write_lock = threading.Lock()
        self.simulate = simulate

        if simulate:
            self.responses.put("SIMULATION MODE: movements are not sent to an Arduino.")
            return

        try:
            import serial
        except ImportError:
            raise RuntimeError("pyserial is required. Install it with: python -m pip install pyserial")

        if not port:
            raise RuntimeError("A serial port is required unless --simulate is used, for example --port COM5.")
        try:
            self.connection = serial.Serial(port, baud, timeout=0.1)
        except serial.SerialException as error:
            raise RuntimeError(f"Could not open or use {port}: {error}") from error

        time.sleep(2.0)  # Most Arduino boards reset when their serial port opens.
        threading.Thread(target=self._read_responses, daemon=True).start()

    def _read_responses(self) -> None:
        while not self.stopped.is_set() and self.connection is not None:
            raw = self.connection.readline()
            if raw:
                self.responses.put(raw.decode(errors="replace").rstrip())

    def send(self, command: str, log: bool = True) -> None:
        if self.simulate:
            if log:
                self.responses.put(f"> {command}")
            return
        if self.connection is None:
            return
        with self.write_lock:
            self.connection.write((command + "\n").encode("ascii"))
            self.connection.flush()
        if log:
            self.responses.put(f"> {command}")

    def close(self) -> None:
        self.stopped.set()
        if self.connection is not None:
            self.connection.close()


class LegVisualizer:
    def __init__(self, transport: SerialTransport, duration: int) -> None:
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.transport = transport
        self.duration = duration
        self.root = tk.Tk()
        self.root.title("Arduino Leg 3D Controller")
        self.root.minsize(940, 650)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.position = list(HOME)
        self.last_valid_position = HOME
        self.dragging = False
        self.last_drag_screen = (0.0, 0.0)
        self.orbiting = False
        self.last_orbit_screen = (0.0, 0.0)
        self.camera_azimuth = math.radians(DEFAULT_CAMERA_AZIMUTH_DEG)
        self.camera_elevation = math.radians(DEFAULT_CAMERA_ELEVATION_DEG)
        self.last_sent_time = 0.0
        self.monitor_walk = False
        self.walk_animation = 0
        self.drag_plane = tk.StringVar(value="YZ")
        self.status = tk.StringVar(value="Drag the yellow foot handle to move the leg.")
        self.axis_values = [tk.DoubleVar(value=value) for value in HOME]
        self.axis_labels = [tk.StringVar(value=f"{value:.1f}") for value in HOME]

        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(0, weight=1)
        self.canvas = tk.Canvas(outer, bg="#121924", highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        controls = ttk.Frame(outer, padding=(8, 4))
        controls.grid(row=0, column=1, sticky="ns")

        ttk.Label(controls, text="Foot Target (mm)", font=("Segoe UI", 12, "bold")).pack(anchor="w")
        for axis, variable, label, limits in zip("XYZ", self.axis_values, self.axis_labels, AXIS_LIMITS):
            row = ttk.Frame(controls)
            row.pack(fill="x", pady=(10, 0))
            ttk.Label(row, text=axis, width=2).pack(side="left")
            ttk.Label(row, textvariable=label, width=8).pack(side="right")
            ttk.Scale(
                controls,
                from_=limits[0],
                to=limits[1],
                variable=variable,
                orient="horizontal",
                length=245,
                command=self._on_slider,
            ).pack(fill="x")

        ttk.Label(controls, text="Drag Plane", font=("Segoe UI", 11, "bold")).pack(anchor="w", pady=(18, 4))
        for plane, detail in (("YZ", "stride + lift"), ("XZ", "reach + lift"), ("XY", "floor plane")):
            ttk.Radiobutton(
                controls, text=f"{plane}  ({detail})", variable=self.drag_plane, value=plane
            ).pack(anchor="w")
        ttk.Label(controls, text="Left drag: foot target\nRight drag: orbit camera").pack(anchor="w", pady=(10, 0))

        buttons = ttk.Frame(controls)
        buttons.pack(fill="x", pady=(20, 8))
        ttk.Button(buttons, text="Home", command=self.home).pack(side="left", padx=(0, 5))
        ttk.Button(buttons, text="Walk", command=self.walk).pack(side="left", padx=5)
        ttk.Button(buttons, text="Stop", command=self.stop).pack(side="left", padx=5)
        ttk.Button(buttons, text="Status", command=lambda: self.transport.send("STATUS")).pack(side="left", padx=5)
        ttk.Button(controls, text="Reset View", command=self.reset_view).pack(anchor="w", pady=(0, 8))

        ttk.Label(controls, text="Arduino Log", font=("Segoe UI", 11, "bold")).pack(anchor="w", pady=(14, 4))
        self.log = tk.Text(controls, width=40, height=13, state="disabled", font=("Consolas", 9))
        self.log.pack(fill="both", expand=True)
        ttk.Label(controls, textvariable=self.status, wraplength=285).pack(anchor="w", pady=(10, 0))

        self.canvas.bind("<Configure>", lambda _event: self.draw())
        self.canvas.bind("<ButtonPress-1>", self._start_drag)
        self.canvas.bind("<B1-Motion>", self._drag)
        self.canvas.bind("<ButtonRelease-1>", self._stop_drag)
        self.canvas.bind("<ButtonPress-3>", self._start_orbit)
        self.canvas.bind("<B3-Motion>", self._orbit)
        self.canvas.bind("<ButtonRelease-3>", self._stop_orbit)
        self.root.after(60, self._poll_responses)
        self.draw()

    def _projection_vectors(self) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
        azimuth = self.camera_azimuth
        elevation = self.camera_elevation
        return (
            (math.cos(azimuth), math.sin(azimuth) * math.sin(elevation)),
            (-math.sin(azimuth), math.cos(azimuth) * math.sin(elevation)),
            (0.0, -math.cos(elevation)),
        )

    def project(self, point: tuple[float, float, float]) -> tuple[float, float]:
        width = max(self.canvas.winfo_width(), 600)
        height = max(self.canvas.winfo_height(), 500)
        scale = min(width / 700.0, height / 530.0)
        origin_x = width * 0.28
        origin_y = height * 0.40
        x, y, z = point
        project_x, project_y, project_z = self._projection_vectors()
        return (
            origin_x + scale * (project_x[0] * x + project_y[0] * y),
            origin_y + scale * (project_x[1] * x + project_y[1] * y + project_z[1] * z),
        )

    def _line(self, first: tuple[float, float, float], second: tuple[float, float, float], **style: object) -> None:
        self.canvas.create_line(*self.project(first), *self.project(second), **style)

    def _joint(self, point: tuple[float, float, float], radius: int, color: str) -> None:
        x, y = self.project(point)
        self.canvas.create_oval(x - radius, y - radius, x + radius, y + radius, fill=color, outline="#e6edf5")

    def draw(self) -> None:
        self.canvas.delete("all")
        ground_z = HOME[2]
        grid_color = "#263648"
        for value in range(-200, 201, 50):
            self._line((80, value, ground_z), (450, value, ground_z), fill=grid_color)
        for value in range(100, 451, 50):
            self._line((value, -200, ground_z), (value, 200, ground_z), fill=grid_color)
        self._line((0, 0, 0), (150, 0, 0), fill="#e96666", width=2, arrow="last")
        self._line((0, 0, 0), (0, 150, 0), fill="#65c985", width=2, arrow="last")
        self._line((0, 0, 0), (0, 0, 140), fill="#609cff", width=2, arrow="last")
        for label, point, color in (("X", (160, 0, 0), "#e96666"), ("Y", (0, 160, 0), "#65c985"), ("Z", (0, 0, 150), "#609cff")):
            self.canvas.create_text(*self.project(point), text=label, fill=color, font=("Segoe UI", 10, "bold"))

        requested = tuple(self.position)
        solution = solve_leg(requested)
        if solution is None:
            solution = solve_leg(self.last_valid_position)
            target_color = "#ff5f62"
            self.status.set("Target is outside the configured reach or servo-angle limits; command not sent.")
        else:
            self.last_valid_position = requested
            target_color = "#ffd34e"
            coxa, femur, tibia = solution.angles
            self.status.set(
                f"Valid target | servo angles: coxa {coxa:.1f}, femur {femur:.1f}, tibia {tibia:.1f} degrees"
            )

        if solution is not None:
            self._line(solution.hip, solution.coxa, fill="#50c8ff", width=6)
            self._line(solution.coxa, solution.knee, fill="#66e0a3", width=6)
            self._line(solution.knee, solution.foot, fill="#ffa451", width=6)
            self._joint(solution.hip, 6, "#50c8ff")
            self._joint(solution.coxa, 6, "#66e0a3")
            self._joint(solution.knee, 7, "#ffa451")

        x, y = self.project(requested)
        self.canvas.create_oval(x - 11, y - 11, x + 11, y + 11, fill=target_color, outline="#ffffff", width=2)
        self.canvas.create_text(x + 15, y - 18, text="drag foot", anchor="w", fill="#ffffff")

    def _set_position(self, position: tuple[float, float, float], send_move: bool, duration: int) -> None:
        self.position = list(position)
        for variable, label, value in zip(self.axis_values, self.axis_labels, position):
            variable.set(round(value, 1))
            label.set(f"{value:.1f}")
        self.draw()
        if send_move and solve_leg(position) is not None:
            self.transport.send(f"MOVE {position[0]:.1f} {position[1]:.1f} {position[2]:.1f} {duration}")

    def _on_slider(self, _value: str) -> None:
        target = tuple(variable.get() for variable in self.axis_values)
        for label, value in zip(self.axis_labels, target):
            label.set(f"{value:.1f}")
        self.position = list(target)
        self.draw()
        if solve_leg(target) is not None:
            self.transport.send(f"MOVE {target[0]:.1f} {target[1]:.1f} {target[2]:.1f} {self.duration}")

    def _start_drag(self, event: object) -> None:
        foot_x, foot_y = self.project(tuple(self.position))
        if math.hypot(event.x - foot_x, event.y - foot_y) <= 18:
            self.dragging = True
            self.last_drag_screen = (event.x, event.y)

    def _drag(self, event: object) -> None:
        if not self.dragging:
            return
        scale = min(max(self.canvas.winfo_width(), 600) / 700.0, max(self.canvas.winfo_height(), 500) / 530.0)
        delta_screen = ((event.x - self.last_drag_screen[0]) / scale, (event.y - self.last_drag_screen[1]) / scale)
        self.last_drag_screen = (event.x, event.y)
        first_axis, second_axis = {"XY": (0, 1), "XZ": (0, 2), "YZ": (1, 2)}[self.drag_plane.get()]
        vectors = self._projection_vectors()
        first_vector = vectors[first_axis]
        second_vector = vectors[second_axis]
        determinant = first_vector[0] * second_vector[1] - second_vector[0] * first_vector[1]
        if abs(determinant) < 0.00001:
            self.status.set("Selected drag plane is edge-on; orbit the camera or choose another plane.")
            return
        first_delta = (delta_screen[0] * second_vector[1] - second_vector[0] * delta_screen[1]) / determinant
        second_delta = (first_vector[0] * delta_screen[1] - delta_screen[0] * first_vector[1]) / determinant
        target = list(self.position)
        target[first_axis] += first_delta
        target[second_axis] += second_delta
        for index, limits in enumerate(AXIS_LIMITS):
            target[index] = max(limits[0], min(limits[1], target[index]))
        now = time.monotonic()
        should_send = now - self.last_sent_time >= 0.04
        if should_send:
            self.last_sent_time = now
        self._set_position(tuple(target), should_send, 60)

    def _stop_drag(self, _event: object) -> None:
        if self.dragging and solve_leg(tuple(self.position)) is not None:
            self.transport.send(f"MOVE {self.position[0]:.1f} {self.position[1]:.1f} {self.position[2]:.1f} 80")
        self.dragging = False

    def _start_orbit(self, event: object) -> None:
        self.orbiting = True
        self.last_orbit_screen = (event.x, event.y)

    def _orbit(self, event: object) -> None:
        if not self.orbiting:
            return
        delta_x = event.x - self.last_orbit_screen[0]
        delta_y = event.y - self.last_orbit_screen[1]
        self.last_orbit_screen = (event.x, event.y)
        self.camera_azimuth = (self.camera_azimuth + delta_x * 0.008) % (2.0 * math.pi)
        self.camera_elevation = (self.camera_elevation - delta_y * 0.008) % (2.0 * math.pi)
        self.draw()

    def _stop_orbit(self, _event: object) -> None:
        self.orbiting = False

    def reset_view(self) -> None:
        self.camera_azimuth = math.radians(DEFAULT_CAMERA_AZIMUTH_DEG)
        self.camera_elevation = math.radians(DEFAULT_CAMERA_ELEVATION_DEG)
        self.draw()

    def home(self) -> None:
        self.monitor_walk = False
        self.walk_animation += 1
        self._set_position(HOME, False, self.duration)
        self.transport.send("HOME")

    def walk(self) -> None:
        self.monitor_walk = True
        self.walk_animation += 1
        self.transport.send("WALK")
        if self.transport.simulate:
            animation = self.walk_animation
            segments = (
                (tuple(self.position), GAIT_FRONT, APPROACH_MS, False),
                (GAIT_FRONT, GAIT_REAR, STANCE_MS, False),
                (GAIT_REAR, GAIT_FRONT, SWING_MS, True),
            )
            self._animate_walk_segment(segments, 0, animation)
        else:
            self.root.after(80, self._poll_walk_position)

    def stop(self) -> None:
        self.monitor_walk = False
        self.walk_animation += 1
        self.transport.send("STOP")

    def _animate_walk_segment(
        self,
        segments: tuple[tuple[tuple[float, float, float], tuple[float, float, float], int, bool], ...],
        index: int,
        animation: int,
        start_time: Optional[float] = None,
    ) -> None:
        if not self.monitor_walk or animation != self.walk_animation:
            return
        if index >= len(segments):
            self._animate_walk_segment(segments, 1, animation)
            return

        start, target, duration, parabolic_swing = segments[index]
        if start_time is None:
            start_time = time.monotonic()
        progress = min(1.0, (time.monotonic() - start_time) * 1000.0 / duration)
        if parabolic_swing:
            position = (
                start[0] + (target[0] - start[0]) * progress,
                start[1] + (target[1] - start[1]) * progress,
                start[2] + (target[2] - start[2]) * progress
                + SWING_HEIGHT * 4.0 * progress * (1.0 - progress),
            )
        else:
            blend = progress * progress * (3.0 - 2.0 * progress)
            position = tuple(first + (second - first) * blend for first, second in zip(start, target))
        self._set_position(position, False, self.duration)

        if progress >= 1.0:
            self._animate_walk_segment(segments, index + 1, animation)
        else:
            self.root.after(16, lambda: self._animate_walk_segment(segments, index, animation, start_time))

    def _poll_walk_position(self) -> None:
        if self.monitor_walk:
            self.transport.send("STATUS", log=False)
            self.root.after(80, self._poll_walk_position)

    def _poll_responses(self) -> None:
        while True:
            try:
                line = self.transport.responses.get_nowait()
            except queue.Empty:
                break
            if line.startswith("POS ") and not self.dragging:
                fields = line.split()
                try:
                    position = tuple(float(value) for value in fields[1:4])
                except (ValueError, IndexError):
                    position = None
                if position is not None and solve_leg(position) is not None:
                    self._set_position(position, False, self.duration)
            elif line == "DONE WALK":
                self.monitor_walk = False
            self.log.configure(state="normal")
            self.log.insert("end", line + "\n")
            self.log.see("end")
            self.log.configure(state="disabled")
        self.root.after(60, self._poll_responses)

    def close(self) -> None:
        self.transport.close()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def interactive_console(transport: SerialTransport) -> None:
    print("Enter Arduino commands such as MOVE 450 0 -30 500, WALK 2, STATUS, HOME, or STOP.")
    print("Press Ctrl+C or enter QUIT to exit.")
    while True:
        while True:
            try:
                print(f"< {transport.responses.get_nowait()}")
            except queue.Empty:
                break
        try:
            command = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not command:
            continue
        if command.upper() in {"QUIT", "EXIT"}:
            return
        transport.send(command)


def main() -> int:
    parser = argparse.ArgumentParser(description="Control and visualize the Arduino leg over USB serial.")
    parser.add_argument("--port", help="Serial port, for example COM5. Not required with --simulate.")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate used by the sketch.")
    parser.add_argument("--simulate", action="store_true", help="Run the visualization without opening a serial port.")
    parser.add_argument("--console", action="store_true", help="Use the original text console instead of the 3D window.")
    parser.add_argument("--move", nargs=3, metavar=("X", "Y", "Z"), type=float, help="Move to one XYZ position.")
    parser.add_argument("--duration", type=int, default=400, help="Move duration in milliseconds.")
    parser.add_argument("--walk", type=int, metavar="CYCLES", help="Run the gait demo this many times.")
    args = parser.parse_args()

    try:
        transport = SerialTransport(args.port, args.baud, args.simulate)
    except RuntimeError as error:
        print(error, file=sys.stderr)
        return 1

    try:
        if args.move:
            x, y, z = args.move
            transport.send(f"MOVE {x:g} {y:g} {z:g} {max(0, args.duration)}")
            time.sleep(max(0.5, args.duration / 1000.0 + 0.25))
        elif args.walk is not None:
            transport.send(f"WALK {max(1, args.walk)}")
            interactive_console(transport)
        elif args.console:
            interactive_console(transport)
        else:
            LegVisualizer(transport, max(0, args.duration)).run()
    finally:
        transport.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
