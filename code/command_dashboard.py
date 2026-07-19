"""Simple button dashboard for sending Arduino leg serial commands."""

from __future__ import annotations

import argparse
import queue
import sys
import threading
import time
from typing import Optional


class SerialCommandLink:
    def __init__(self) -> None:
        self.connection = None
        self.responses: queue.Queue[str] = queue.Queue()
        self.stopped = threading.Event()
        self.write_lock = threading.Lock()
        self.simulate = False

    def connect(self, port: str, baud: int, simulate: bool) -> None:
        self.close()
        self.simulate = simulate
        self.stopped.clear()

        if simulate:
            self.responses.put("SIMULATION MODE: commands are logged but not sent.")
            return

        try:
            import serial
        except ImportError as error:
            raise RuntimeError("pyserial is required. Install it with: python -m pip install pyserial") from error

        if not port.strip():
            raise RuntimeError("Enter a serial port, for example COM5.")

        try:
            self.connection = serial.Serial(port.strip(), baud, timeout=0.1)
        except serial.SerialException as error:
            raise RuntimeError(f"Could not open {port}: {error}") from error

        time.sleep(2.0)  # Most Arduino boards reset when serial opens.
        threading.Thread(target=self._read_loop, daemon=True).start()
        self.responses.put(f"CONNECTED {port.strip()} @ {baud}")

    def _read_loop(self) -> None:
        while not self.stopped.is_set() and self.connection is not None:
            raw = self.connection.readline()
            if raw:
                self.responses.put(raw.decode(errors="replace").rstrip())

    def send(self, command: str) -> None:
        command = command.strip()
        if not command:
            return
        if self.simulate:
            self.responses.put(f"> {command}")
            return
        if self.connection is None:
            self.responses.put("ERR not connected")
            return
        with self.write_lock:
            self.connection.write((command + "\n").encode("ascii"))
            self.connection.flush()
        self.responses.put(f"> {command}")

    def close(self) -> None:
        self.stopped.set()
        if self.connection is not None:
            self.connection.close()
        self.connection = None


class CommandDashboard:
    def __init__(self, initial_port: str, baud: int, simulate: bool) -> None:
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.ttk = ttk
        self.link = SerialCommandLink()
        self.root = tk.Tk()
        self.root.title("Leg Prototype Command Dashboard")
        self.root.minsize(760, 590)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.port = tk.StringVar(value=initial_port)
        self.baud = tk.IntVar(value=baud)
        self.simulate = tk.BooleanVar(value=simulate)
        self.raw_command = tk.StringVar()

        self.move_x = tk.DoubleVar(value=450.0)
        self.move_y = tk.DoubleVar(value=0.0)
        self.move_z = tk.DoubleVar(value=-30.0)
        self.move_ms = tk.IntVar(value=400)
        self.walk_cycles = tk.StringVar(value="")
        self.raw_coxa = tk.IntVar(value=90)
        self.raw_femur = tk.IntVar(value=90)
        self.raw_tibia = tk.IntVar(value=90)

        self.joint = tk.StringVar(value="COXA")
        self.joint_angle = tk.IntVar(value=0)
        self.raw_joint_angle = tk.IntVar(value=90)

        self._build_ui()
        self.root.after(60, self._poll_responses)

        if simulate:
            self.connect()

    def _build_ui(self) -> None:
        ttk = self.ttk

        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)
        outer.columnconfigure(0, weight=1)
        outer.columnconfigure(1, weight=1)
        outer.rowconfigure(2, weight=1)

        connection = ttk.LabelFrame(outer, text="Connection", padding=8)
        connection.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        connection.columnconfigure(1, weight=1)
        ttk.Label(connection, text="Port").grid(row=0, column=0, padx=(0, 6))
        ttk.Entry(connection, textvariable=self.port, width=16).grid(row=0, column=1, sticky="w")
        ttk.Label(connection, text="Baud").grid(row=0, column=2, padx=(12, 6))
        ttk.Entry(connection, textvariable=self.baud, width=8).grid(row=0, column=3, sticky="w")
        ttk.Checkbutton(connection, text="Simulate", variable=self.simulate).grid(row=0, column=4, padx=12)
        ttk.Button(connection, text="Connect", command=self.connect).grid(row=0, column=5, padx=(0, 6))
        ttk.Button(connection, text="Disconnect", command=self.disconnect).grid(row=0, column=6)

        ik = ttk.LabelFrame(outer, text="IK Controller Sketch", padding=8)
        ik.grid(row=1, column=0, sticky="nsew", padx=(0, 5))
        for index, (label, variable) in enumerate(
            (("X", self.move_x), ("Y", self.move_y), ("Z", self.move_z), ("ms", self.move_ms))
        ):
            ttk.Label(ik, text=label).grid(row=0, column=index * 2, padx=(0, 4), pady=(0, 6))
            ttk.Entry(ik, textvariable=variable, width=8).grid(row=0, column=index * 2 + 1, padx=(0, 8), pady=(0, 6))
        ttk.Button(ik, text="Move XYZ", command=self.send_move).grid(row=1, column=0, columnspan=2, sticky="ew")
        ttk.Button(ik, text="Home", command=lambda: self.send("HOME")).grid(row=1, column=2, columnspan=2, sticky="ew", padx=4)
        ttk.Button(ik, text="Stop", command=lambda: self.send("STOP")).grid(row=1, column=4, columnspan=2, sticky="ew", padx=4)
        ttk.Button(ik, text="Status", command=lambda: self.send("STATUS")).grid(row=1, column=6, columnspan=2, sticky="ew")

        ttk.Label(ik, text="Walk cycles blank = continuous").grid(row=2, column=0, columnspan=3, sticky="w", pady=(10, 0))
        ttk.Entry(ik, textvariable=self.walk_cycles, width=8).grid(row=2, column=3, sticky="w", pady=(10, 0))
        ttk.Button(ik, text="Walk", command=self.send_walk).grid(row=2, column=4, columnspan=2, sticky="ew", padx=4, pady=(10, 0))
        ttk.Button(ik, text="Help", command=lambda: self.send("HELP")).grid(row=2, column=6, columnspan=2, sticky="ew", pady=(10, 0))
        ttk.Button(ik, text="Center Raw 90/90/90", command=lambda: self.send("CENTER")).grid(
            row=3, column=0, columnspan=3, sticky="ew", pady=(10, 0)
        )
        for index, (label, variable) in enumerate(
            (("Coxa", self.raw_coxa), ("Femur", self.raw_femur), ("Tibia", self.raw_tibia))
        ):
            ttk.Label(ik, text=label).grid(row=4, column=index * 2, padx=(0, 4), pady=(8, 0))
            ttk.Entry(ik, textvariable=variable, width=7).grid(row=4, column=index * 2 + 1, padx=(0, 8), pady=(8, 0))
        ttk.Button(ik, text="Raw Angles", command=self.send_raw_angles).grid(
            row=4, column=6, columnspan=2, sticky="ew", pady=(8, 0)
        )

        joint = ttk.LabelFrame(outer, text="Joint Test Sketch", padding=8)
        joint.grid(row=1, column=1, sticky="nsew", padx=(5, 0))
        ttk.Button(joint, text="Center All", command=lambda: self.send("CENTER")).grid(row=0, column=0, sticky="ew", padx=3, pady=3)
        ttk.Button(joint, text="Test All", command=lambda: self.send("TEST ALL")).grid(row=0, column=1, sticky="ew", padx=3, pady=3)
        ttk.Button(joint, text="Test Coxa", command=lambda: self.send("TEST COXA")).grid(row=1, column=0, sticky="ew", padx=3, pady=3)
        ttk.Button(joint, text="Test Femur", command=lambda: self.send("TEST FEMUR")).grid(row=1, column=1, sticky="ew", padx=3, pady=3)
        ttk.Button(joint, text="Test Tibia", command=lambda: self.send("TEST TIBIA")).grid(row=1, column=2, sticky="ew", padx=3, pady=3)

        ttk.Label(joint, text="Joint").grid(row=2, column=0, sticky="w", pady=(12, 0))
        ttk.Combobox(joint, textvariable=self.joint, values=("COXA", "FEMUR", "TIBIA"), width=9, state="readonly").grid(
            row=2, column=1, sticky="w", pady=(12, 0)
        )
        ttk.Label(joint, text="Physical angle").grid(row=3, column=0, sticky="w", pady=(6, 0))
        ttk.Scale(joint, from_=-90, to=90, orient="horizontal", variable=self.joint_angle, command=self._round_joint_angle).grid(
            row=3, column=1, columnspan=2, sticky="ew", pady=(6, 0)
        )
        ttk.Entry(joint, textvariable=self.joint_angle, width=8).grid(row=4, column=1, sticky="w", pady=(6, 0))
        ttk.Button(joint, text="Set Physical Angle", command=self.send_joint_set).grid(
            row=4, column=2, sticky="ew", padx=3, pady=(6, 0)
        )
        ttk.Label(joint, text="Raw servo angle").grid(row=5, column=0, sticky="w", pady=(12, 0))
        ttk.Scale(joint, from_=0, to=180, orient="horizontal", variable=self.raw_joint_angle, command=self._round_raw_joint_angle).grid(
            row=5, column=1, columnspan=2, sticky="ew", pady=(12, 0)
        )
        ttk.Entry(joint, textvariable=self.raw_joint_angle, width=8).grid(row=6, column=1, sticky="w", pady=(6, 0))
        ttk.Button(joint, text="Set Raw Servo Angle", command=self.send_joint_raw).grid(
            row=6, column=2, sticky="ew", padx=3, pady=(6, 0)
        )
        joint.columnconfigure(0, weight=1)
        joint.columnconfigure(1, weight=1)
        joint.columnconfigure(2, weight=1)

        log_frame = ttk.LabelFrame(outer, text="Serial Log / Raw Command", padding=8)
        log_frame.grid(row=2, column=0, columnspan=2, sticky="nsew", pady=(10, 0))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log = self.tk.Text(log_frame, height=16, state="disabled", font=("Consolas", 9))
        self.log.grid(row=0, column=0, columnspan=3, sticky="nsew", pady=(0, 8))
        ttk.Entry(log_frame, textvariable=self.raw_command).grid(row=1, column=0, sticky="ew", padx=(0, 6))
        ttk.Button(log_frame, text="Send Raw", command=self.send_raw).grid(row=1, column=1, padx=(0, 6))
        ttk.Button(log_frame, text="Clear Log", command=self.clear_log).grid(row=1, column=2)

    def connect(self) -> None:
        try:
            self.link.connect(self.port.get(), int(self.baud.get()), self.simulate.get())
        except Exception as error:
            self._append_log(f"ERR {error}")

    def disconnect(self) -> None:
        self.link.close()
        self._append_log("DISCONNECTED")

    def send(self, command: str) -> None:
        self.link.send(command)

    def send_move(self) -> None:
        self.send(f"MOVE {self.move_x.get():g} {self.move_y.get():g} {self.move_z.get():g} {int(self.move_ms.get())}")

    def send_walk(self) -> None:
        cycles = self.walk_cycles.get().strip()
        self.send("WALK" if not cycles else f"WALK {cycles}")

    def send_raw_angles(self) -> None:
        coxa = max(0, min(180, int(self.raw_coxa.get())))
        femur = max(0, min(180, int(self.raw_femur.get())))
        tibia = max(0, min(180, int(self.raw_tibia.get())))
        self.raw_coxa.set(coxa)
        self.raw_femur.set(femur)
        self.raw_tibia.set(tibia)
        self.send(f"RAW {coxa} {femur} {tibia}")

    def send_joint_set(self) -> None:
        angle = int(self.joint_angle.get())
        self.joint_angle.set(angle)
        self.send(f"SET {self.joint.get()} {angle}")

    def send_joint_raw(self) -> None:
        angle = max(0, min(180, int(self.raw_joint_angle.get())))
        self.raw_joint_angle.set(angle)
        self.send(f"RAW {self.joint.get()} {angle}")

    def send_raw(self) -> None:
        self.send(self.raw_command.get())
        self.raw_command.set("")

    def clear_log(self) -> None:
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    def _round_joint_angle(self, _value: str) -> None:
        self.joint_angle.set(int(round(self.joint_angle.get())))

    def _round_raw_joint_angle(self, _value: str) -> None:
        self.raw_joint_angle.set(int(round(self.raw_joint_angle.get())))

    def _append_log(self, line: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", line + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _poll_responses(self) -> None:
        while True:
            try:
                line = self.link.responses.get_nowait()
            except queue.Empty:
                break
            self._append_log(line)
        self.root.after(60, self._poll_responses)

    def close(self) -> None:
        self.link.close()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> int:
    parser = argparse.ArgumentParser(description="Simple command dashboard for the Arduino leg prototype.")
    parser.add_argument("--port", default="", help="Serial port, for example COM5.")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate.")
    parser.add_argument("--simulate", action="store_true", help="Open without serial hardware and log commands only.")
    args = parser.parse_args()

    try:
        CommandDashboard(args.port, args.baud, args.simulate).run()
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        print(error, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
