#!/usr/bin/env python3
import argparse
import csv
import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

FRAME_LEN = 49
HEAD = b"\xaa\x55"
FIELDS = ["time_ms", "seq", "angle", "location", "angle_out", "pos_out", "run_state", "center_angle", "pos_target", "pos_set", "bar_vel", "target_vel", "motor_cmd", "raw_code", "avg4", "mov_active"]

def parse_frame(f):
    if len(f) != FRAME_LEN or f[:2] != HEAD or f[3] != FRAME_LEN or f[48] != 0x5A:
        return None
    crc = 0xFFFF
    for v in f[2:46]:
        crc ^= v << 8
        for _ in range(8): crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    if struct.unpack_from("<H", f, 46)[0] != crc: return None
    specs = [("time_ms", "I"), ("seq", "H"), ("angle", "H"), ("location", "i"), ("angle_out", "h"), ("pos_out", "h"), ("run_state", "B"), ("center_angle", "H"), ("pos_target", "i"), ("pos_set", "i"), ("bar_vel", "i"), ("target_vel", "i"), ("motor_cmd", "h"), ("raw_code", "H"), ("avg4", "H")]
    row = {}
    off = 4
    for name, fmt in specs:
        row[name] = struct.unpack_from("<" + fmt, f, off)[0]; off += struct.calcsize(fmt)
    row["mov_active"] = f[45] & 1
    return row

class App:
    def __init__(self, root, port):
        self.root, self.port, self.q, self.rows, self.running = root, port, queue.Queue(), [], True
        root.title("FPGA Pendulum Telemetry")
        bar = ttk.Frame(root); bar.pack(fill="x")
        ttk.Label(bar, text="Port:").pack(side="left")
        self.port_var = tk.StringVar(value=port); ttk.Entry(bar, textvariable=self.port_var, width=12).pack(side="left")
        ttk.Button(bar, text="Connect", command=self.connect).pack(side="left")
        ttk.Button(bar, text="Save CSV", command=self.save).pack(side="left")
        self.status = tk.StringVar(value="Disconnected"); ttk.Label(bar, textvariable=self.status).pack(side="left", padx=8)
        try:
            import matplotlib.pyplot as plt
            from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
            from matplotlib.figure import Figure
            self.fig = Figure(figsize=(10, 6), dpi=100); self.axes = self.fig.subplots(2, 2); self.canvas = FigureCanvasTkAgg(self.fig, root); self.canvas.get_tk_widget().pack(fill="both", expand=True)
        except ImportError: raise SystemExit("Install matplotlib: pip install matplotlib")
        self.connect(); self.update()

    def connect(self):
        self.running = True
        threading.Thread(target=self.reader, args=(self.port_var.get(),), daemon=True).start()

    def reader(self, port):
        try:
            import serial
            ser = serial.Serial(port, 115200, timeout=.5); buf = b""; self.status.set("Connected")
            while self.running:
                buf += ser.read(256)
                while len(buf) >= FRAME_LEN:
                    i = buf.find(HEAD)
                    if i < 0: buf = buf[-1:]; break
                    if i: buf = buf[i:]
                    if len(buf) < FRAME_LEN: break
                    row = parse_frame(buf[:FRAME_LEN]); buf = buf[FRAME_LEN:]
                    if row: self.q.put(row)
            ser.close()
        except Exception as e: self.status.set(str(e))

    def update(self):
        while not self.q.empty(): self.rows.append(self.q.get())
        data = self.rows[-300:]
        if data:
            x = [r["time_ms"] / 1000 for r in data]
            for ax in self.axes: ax.clear(); ax.grid(True, alpha=.3)
            self.axes[0,0].plot(x, [r["location"] for r in data], label="position"); self.axes[0,0].plot(x, [r["pos_target"] for r in data], label="trajectory target"); self.axes[0,0].legend(); self.axes[0,0].set_title("Position")
            self.axes[0,1].plot(x, [r["bar_vel"] for r in data], label="measured"); self.axes[0,1].plot(x, [r["target_vel"] for r in data], label="commanded"); self.axes[0,1].legend(); self.axes[0,1].set_title("Speed (count/s)")
            self.axes[1,0].plot(x, [r["angle"] for r in data]); self.axes[1,0].set_title("Angle code")
            self.axes[1,1].plot(x, [r["motor_cmd"] for r in data], label="motor"); self.axes[1,1].plot(x, [r["angle_out"] for r in data], label="angle PID"); self.axes[1,1].legend(); self.axes[1,1].set_title("Control")
            self.fig.tight_layout(); self.canvas.draw_idle()
        self.root.after(100, self.update)

    def save(self):
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")])
        if path:
            with open(path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=FIELDS); w.writeheader(); w.writerows(self.rows)
            messagebox.showinfo("Saved", path)

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("port", help="COM port, e.g. COM3"); args = ap.parse_args()
    root = tk.Tk(); App(root, args.port); root.mainloop()
