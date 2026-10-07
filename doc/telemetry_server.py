#!/usr/bin/env python3
"""Read the FPGA UART with pyserial and stream parsed frames to the web dashboard."""
import argparse
import json
import queue
import struct
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

FRAME_LEN = 49
HEAD = b"\xAA\x55"
TAIL = 0x5A


def parse_frame(frame):
    if len(frame) != FRAME_LEN or frame[:2] != HEAD or frame[2] != 2 or frame[3] != FRAME_LEN or frame[48] != TAIL:
        return None
    crc = 0xFFFF
    for value in frame[2:46]:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    if struct.unpack_from("<H", frame, 46)[0] != crc:
        return None
    specs = [("time_ms", "I"), ("seq", "H"), ("angle", "H"), ("location", "i"),
             ("angle_out", "h"), ("pos_out", "h"), ("run_state", "B"),
             ("center_angle", "H"), ("pos_target", "i"), ("pos_set", "i"),
             ("bar_vel", "i"), ("target_vel", "i"), ("motor_cmd", "h"),
             ("raw_code", "H"), ("avg4", "H")]
    row, offset = {}, 4
    for name, fmt in specs:
        row[name] = struct.unpack_from("<" + fmt, frame, offset)[0]
        offset += struct.calcsize(fmt)
    row["mov_active"] = frame[45] & 1
    return row


class State:
    def __init__(self):
        self.clients = []
        self.lock = threading.Lock()
        self.good = 0
        self.bad = 0

    def add(self):
        q = queue.Queue(maxsize=20)
        with self.lock:
            self.clients.append(q)
        return q

    def remove(self, q):
        with self.lock:
            if q in self.clients:
                self.clients.remove(q)

    def publish(self, row):
        payload = json.dumps(row, separators=(",", ":"), ensure_ascii=False)
        with self.lock:
            clients = list(self.clients)
        for q in clients:
            try:
                q.put_nowait(payload)
            except queue.Full:
                try:
                    q.get_nowait()
                    q.put_nowait(payload)
                except queue.Empty:
                    pass


class Handler(BaseHTTPRequestHandler):
    state = None
    web_root = None

    def log_message(self, fmt, *args):
        print(fmt % args)

    def do_GET(self):
        if self.path.split("?", 1)[0] == "/events":
            self.events()
            return
        rel = self.path.split("?", 1)[0].lstrip("/") or "telemetry.html"
        target = (self.web_root / rel).resolve()
        if self.web_root not in target.parents and target != self.web_root:
            self.send_error(403)
            return
        if not target.is_file():
            self.send_error(404)
            return
        data = target.read_bytes()
        content_type = "text/html; charset=utf-8" if target.suffix == ".html" else "text/plain; charset=utf-8"
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def events(self):
        q = self.state.add()
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        try:
            self.wfile.write(b": connected\n\n")
            self.wfile.flush()
            while True:
                try:
                    payload = q.get(timeout=15)
                    self.wfile.write(("data: " + payload + "\n\n").encode("utf-8"))
                except queue.Empty:
                    self.wfile.write(b": heartbeat\n\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            self.state.remove(q)


def serial_loop(port_name, state):
    try:
        import serial
    except ImportError:
        print("Missing pyserial. Install it with: python -m pip install pyserial")
        return
    print(f"Opening {port_name} at 115200 8N1")
    try:
        with serial.Serial(port_name, 115200, timeout=0.2) as ser:
            buf = bytearray()
            while True:
                chunk = ser.read(256)
                if chunk:
                    buf.extend(chunk)
                while len(buf) >= FRAME_LEN:
                    pos = buf.find(HEAD)
                    if pos < 0:
                        del buf[:-1]
                        break
                    if pos:
                        del buf[:pos]
                    if len(buf) < FRAME_LEN:
                        break
                    row = parse_frame(bytes(buf[:FRAME_LEN]))
                    del buf[:FRAME_LEN]
                    if row is None:
                        state.bad += 1
                    else:
                        state.good += 1
                        state.publish(row)
    except Exception as exc:
        print(f"Serial error: {exc}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="COM port, for example COM3")
    parser.add_argument("--http", type=int, default=8001)
    args = parser.parse_args()
    state = State()
    handler = Handler
    handler.state = state
    handler.web_root = Path(__file__).resolve().parents[1] / "web"
    threading.Thread(target=serial_loop, args=(args.port, state), daemon=True).start()
    print(f"Open http://localhost:{args.http}/telemetry.html?bridge=1")
    ThreadingHTTPServer(("127.0.0.1", args.http), handler).serve_forever()


if __name__ == "__main__":
    main()
