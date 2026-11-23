"""Interactive calibration subprocess wrapper for Arm101 Studio.

Patches `builtins.input`, `enter_pressed()`, `move_cursor_up()`, and follower torque
handling so that `lerobot-calibrate`:
1. Works seamlessly over piped stdin/stdout on both Windows and Linux.
2. Immediately disables follower torque after `connect(calibrate=False)` so the arm
   NEVER remains stiff (僵直) while waiting for human positioning or confirmation.
3. Emits flushed `[CAL_STAGE]` progress lines so the Studio UI never stays stuck on
   "正在连接..." without feedback.
4. Cleanly disables torque and disconnects on EOF / stop / signal exit.
"""

from __future__ import annotations

import builtins
import os
import queue
import signal
import sys
import threading
from typing import Any


def drain_stdin_queue(stdin_queue: queue.Queue[str | None]) -> None:
    """Drain any stale buffered newline strings while preserving an EOF (None) sentinel."""
    eof_seen = False
    while True:
        try:
            item = stdin_queue.get_nowait()
        except queue.Empty:
            break
        if item is None:
            eof_seen = True
            break
    if eof_seen:
        stdin_queue.put(None)


def install_piped_stdin_shims(stdin_queue: queue.Queue[str | None]) -> None:
    """Install cross-platform piped stdin shims onto builtins and lerobot helpers."""
    import lerobot.motors.motors_bus as motors_bus_mod
    import lerobot.utils.utils as lerobot_utils_mod

    def _piped_input(prompt: str = "") -> str:
        clean_prompt = " ".join(str(prompt).splitlines()).strip()
        if clean_prompt:
            print(f"[CAL_PROMPT] {clean_prompt}", flush=True)
        else:
            print("[CAL_PROMPT] Press ENTER to continue...", flush=True)
        item = stdin_queue.get()
        if item is None:
            raise EOFError("Calibration stdin closed.")
        # Drain any duplicate/stale enters sent before Step 2 range recording begins
        drain_stdin_queue(stdin_queue)
        return item.rstrip("\r\n")

    def _piped_enter_pressed() -> bool:
        try:
            item = stdin_queue.get_nowait()
        except queue.Empty:
            return False
        if item is None:
            raise EOFError("Calibration stdin closed.")
        return True

    def _noop_move_cursor_up(lines: int) -> None:
        _ = lines

    builtins.input = _piped_input  # type: ignore[assignment]
    lerobot_utils_mod.enter_pressed = _piped_enter_pressed
    lerobot_utils_mod.move_cursor_up = _noop_move_cursor_up
    motors_bus_mod.enter_pressed = _piped_enter_pressed
    motors_bus_mod.move_cursor_up = _noop_move_cursor_up
    motors_bus_mod.flush_stdin_queue = lambda: drain_stdin_queue(stdin_queue)  # type: ignore[attr-defined]


def install_calibration_safety_hooks() -> None:
    """Ensure SO-101 follower torque is released immediately after connect() during calibration."""
    try:
        from lerobot.robots.robot import Robot
        from lerobot.robots.so_follower.so_follower import SOFollower
    except Exception:
        return

    if not getattr(SOFollower.connect, "_studio_cal_hooked", False):
        orig_connect = SOFollower.connect

        def _safe_cal_connect(self: Any, calibrate: bool = True) -> None:
            print(f"[CAL_STAGE] connecting_bus port={getattr(self.config, 'port', '')}", flush=True)
            orig_connect(self, calibrate=calibrate)
            # SOFollower.configure() exits torque_disabled() which re-enables torque (arm becomes stiff).
            # When calibrating (calibrate=False from lerobot-calibrate), release torque immediately
            # so the user can move the arm freely and it never locks up stiff while waiting for input.
            if not calibrate:
                try:
                    self.bus.disable_torque()
                    print("[CAL_STAGE] torque_disabled", flush=True)
                except Exception as exc:
                    print(f"[CAL_STAGE] torque_disable_warn {exc}", flush=True)

        _safe_cal_connect._studio_cal_hooked = True  # type: ignore[attr-defined]
        SOFollower.connect = _safe_cal_connect  # type: ignore[method-assign]

    if not getattr(Robot._keep_calibration_file, "_studio_cal_hooked", False):
        orig_keep = Robot._keep_calibration_file

        def _safe_keep_cal(self: Any, bus: Any) -> bool:
            try:
                bus.disable_torque()
            except Exception:
                pass
            return orig_keep(self, bus)

        _safe_keep_cal._studio_cal_hooked = True  # type: ignore[attr-defined]
        Robot._keep_calibration_file = _safe_keep_cal  # type: ignore[method-assign]


def main() -> int:
    print("[CAL_STAGE] starting", flush=True)
    stdin_queue: queue.Queue[str | None] = queue.Queue()

    def _reader() -> None:
        # Read raw fd 0 via os.read() instead of sys.stdin.readline() to avoid
        # Windows CRT / Python TextIOWrapper lock deadlocks during lerobot imports.
        buf = b""
        try:
            while True:
                chunk = os.read(0, 4096)
                if not chunk:
                    if buf:
                        stdin_queue.put(buf.decode("utf-8", errors="replace"))
                    stdin_queue.put(None)
                    break
                buf += chunk
                while b"\n" in buf:
                    raw_line, buf = buf.split(b"\n", 1)
                    stdin_queue.put(raw_line.decode("utf-8", errors="replace") + "\n")
        except Exception:
            stdin_queue.put(None)

    def _on_signal(_signum: int, _frame: Any) -> None:
        stdin_queue.put(None)
        raise KeyboardInterrupt("Calibration interrupted by stop signal.")

    for sig_name in ("SIGINT", "SIGTERM", "SIGBREAK"):
        sig = getattr(signal, sig_name, None)
        if sig is not None:
            try:
                signal.signal(sig, _on_signal)
            except Exception:
                pass

    # Import lerobot BEFORE starting the stdin reader thread to prevent Windows import/stdio deadlocks
    install_piped_stdin_shims(stdin_queue)
    install_calibration_safety_hooks()
    from lerobot.scripts.lerobot_calibrate import main as lerobot_calibrate_main

    threading.Thread(target=_reader, name="cal-stdin-reader", daemon=True).start()

    try:
        lerobot_calibrate_main()
    except (EOFError, KeyboardInterrupt):
        return 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
