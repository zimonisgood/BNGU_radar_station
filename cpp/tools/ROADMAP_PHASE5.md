# Phase 5+ Roadmap (post-MVP)

MVP is done: camera capture + dual YOLO + perspective mapping + bench.

Next, in order:

1. **Filters** — port sliding-window and hybrid gated Kalman from Python `main.py`
2. **Blind-zone prediction** — fixed points + mask landing checks
3. **Referee serial** — CRC + packets from `RM_serial_py` (incl. 0x0305 coords)
4. **Double vulnerability** — distance + motion-trend decision
5. **Recording** — frame dump; optional Qt calibration tool (Python calib remains OK)

Keep Python `calibration.py` for producing `.npy` / YAML until a Qt port exists.
