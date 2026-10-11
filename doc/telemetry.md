# UART telemetry

The FPGA sends a 49-byte little-endian frame every 50 ms at 115200 8N1. The UART frame transmitter has a `FRAME_MS` parameter. This is independent of the control loop periods: the position loop and measured speed remain 50 ms, while the angle loop and motor command remain 5 ms.

At 115200 8N1, one 49-byte frame takes about 4.25 ms on the wire. The 50 ms setting uses about 8.5% of the link and leaves ample idle time. Data sent while the browser is disconnected cannot be recovered because the FPGA transmitter has no receive-side queue; the sequence and CRC fields let the viewer detect such gaps.

| Bytes | Field |
|---|---|
| 0..1 | `AA 55` |
| 2 | version `2` |
| 3 | frame length `49` |
| 4..7 | uptime in ms (`uint32`) |
| 8..9 | sequence (`uint16`) |
| 10..11 | angle |
| 12..15 | measured position (count) |
| 16..17 | angle-loop output |
| 18..19 | position-loop output |
| 20 | run state |
| 21..22 | center angle |
| 23..26 | trajectory position target |
| 27..30 | requested final position |
| 31..34 | measured speed (count/s) |
| 35..38 | commanded speed (count/s) |
| 39..40 | motor command |
| 41..42 | ADC raw code |
| 43..44 | ADC averaged code |
| 45 | motion-active flag |
| 46..47 | CRC-16/CCITT-FALSE over bytes 2..45 |
| 48 | `5A` |

`doc/telemetry_gui.py` provides a live plot for position, measured/commanded speed, angle, and motor command. It can save the received samples as CSV.

## Web presentation

Open `web/telemetry.html` in Chrome or Edge. The page uses Web Serial directly and presents these ten channels:

`time_s, angle, location, pos_target, pos_set, bar_vel, target_vel, motor_cmd, angle_out, pos_out`

The page parses the existing CRC-protected FPGA frame, so the FPGA does not need a floating-point UART formatter.

## Browser compatibility fallback

Some USB-UART drivers report every read as `BreakError` to Chrome Web Serial even though SSCOM receives the bytes correctly. Use the Python bridge in that case:

```powershell
python -m pip install pyserial
python doc/telemetry_server.py COM3 --http 8001
```

Open `http://localhost:8001/telemetry.html?bridge=1`. The Python process owns the COM port and the browser only receives parsed frames over local SSE, so SSCOM must be closed while the bridge is running.
