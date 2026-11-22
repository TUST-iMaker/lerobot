"""Interactive calibration subprocess wrapper for Arm101 Studio.

Patches `builtins.input`, `enter_pressed()`, and `move_cursor_up()` so that
`lerobot-calibrate` works seamlessly over piped stdin/stdout on both Windows and Linux.
"""

from __future__ import annotations

import builtins
import queue
import sys
import threading


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
        if prompt:
            print(f"[CAL_PROMPT] {prompt}", flush=True)
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


def main() -> int:
    stdin_queue: queue.Queue[str | None] = queue.Queue()

    def _reader() -> None:
        try:
            while True:
                line = sys.stdin.readline()
                if line == "":
                    stdin_queue.put(None)
                    break
                stdin_queue.put(line)
        except Exception:
            stdin_queue.put(None)

    threading.Thread(target=_reader, name="cal-stdin-reader", daemon=True).start()
    install_piped_stdin_shims(stdin_queue)

    from lerobot.scripts.lerobot_calibrate import main as lerobot_calibrate_main

    lerobot_calibrate_main()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
