"""Tracked subprocess lifecycle manager guaranteeing zero orphan child processes."""

from __future__ import annotations

import atexit
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class ManagedProcess:
    """State wrapper for an isolated child process tree."""

    process_id: str
    name: str
    cmd: list[str]
    proc: subprocess.Popen[str]
    started_at: float
    log_tail: deque[str] = field(default_factory=lambda: deque(maxlen=300))
    log_file: Path | None = None
    returncode: int | None = None
    reader_thread: threading.Thread | None = None


class ProcessRegistry:
    """Spawns isolated process groups and reaps entire process trees cleanly."""

    def __init__(self, logs_dir: Path | None = None) -> None:
        self.logs_dir = logs_dir
        self._lock = threading.RLock()
        self._processes: dict[str, ManagedProcess] = {}
        atexit.register(self.terminate_all)

    def spawn(
        self,
        process_id: str,
        name: str,
        cmd: list[str],
        *,
        cwd: Path | None = None,
        env_overrides: dict[str, str] | None = None,
        on_line: Callable[[str], None] | None = None,
        stdin_pipe: bool = False,
    ) -> ManagedProcess:
        """Spawn a subprocess in its own OS process group."""
        with self._lock:
            existing = self._processes.get(process_id)
            if existing is not None and existing.proc.poll() is None:
                raise RuntimeError(f"Process '{process_id}' is already running (pid={existing.proc.pid}).")

            env = os.environ.copy()
            env["PYTHONUNBUFFERED"] = "1"
            env["PYTHONIOENCODING"] = "utf-8"
            if env_overrides:
                env.update(env_overrides)

            popen_kwargs: dict[str, object] = {
                "stdout": subprocess.PIPE,
                "stderr": subprocess.STDOUT,
                "stdin": subprocess.PIPE if stdin_pipe else subprocess.DEVNULL,
                "text": True,
                "encoding": "utf-8",
                "errors": "replace",
                "bufsize": 1,
                "env": env,
            }
            if cwd is not None:
                popen_kwargs["cwd"] = str(cwd)

            if sys.platform == "win32":
                creationflags = subprocess.CREATE_NEW_PROCESS_GROUP | getattr(
                    subprocess, "CREATE_NO_WINDOW", 0
                )
                popen_kwargs["creationflags"] = creationflags
            else:
                popen_kwargs["start_new_session"] = True

            proc = subprocess.Popen(cmd, **popen_kwargs)  # type: ignore[arg-type]

            log_file: Path | None = None
            if self.logs_dir is not None:
                self.logs_dir.mkdir(parents=True, exist_ok=True)
                safe_name = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in name)
                log_file = self.logs_dir / f"{safe_name}.log"

            managed = ManagedProcess(
                process_id=process_id,
                name=name,
                cmd=list(cmd),
                proc=proc,
                started_at=time.time(),
                log_file=log_file,
            )

            def _pump_output() -> None:
                fh = None
                try:
                    if log_file is not None:
                        fh = log_file.open("a", encoding="utf-8")
                    if proc.stdout is not None:
                        for raw_line in iter(proc.stdout.readline, ""):
                            line = raw_line.rstrip("\r\n")
                            if not line:
                                continue
                            with self._lock:
                                managed.log_tail.append(line)
                            if fh is not None:
                                fh.write(line + "\n")
                                fh.flush()
                            if on_line is not None:
                                try:
                                    on_line(line)
                                except Exception:
                                    pass
                except Exception:
                    pass
                finally:
                    if fh is not None:
                        try:
                            fh.close()
                        except Exception:
                            pass
                    try:
                        managed.returncode = proc.wait(timeout=1.0)
                    except Exception:
                        managed.returncode = proc.poll()

            reader = threading.Thread(
                target=_pump_output, name=f"proc-log-{process_id}", daemon=True
            )
            managed.reader_thread = reader
            self._processes[process_id] = managed
            reader.start()
            return managed

    def is_running(self, process_id: str) -> bool:
        with self._lock:
            managed = self._processes.get(process_id)
            if managed is None:
                return False
            rc = managed.proc.poll()
            if rc is not None:
                managed.returncode = rc
                return False
            return True

    def get(self, process_id: str) -> ManagedProcess | None:
        with self._lock:
            return self._processes.get(process_id)

    def write_stdin(self, process_id: str, text: str = "") -> bool:
        """Write a line to a running interactive subprocess stdin."""
        with self._lock:
            managed = self._processes.get(process_id)
            if managed is None or managed.proc.poll() is not None or managed.proc.stdin is None:
                return False
            payload = text if text.endswith("\n") else f"{text}\n"
            try:
                managed.proc.stdin.write(payload)
                managed.proc.stdin.flush()
                return True
            except Exception:
                return False

    def terminate(self, process_id: str, timeout_s: float = 2.0) -> bool:
        """Terminate a process and all its descendants cleanly."""
        with self._lock:
            managed = self._processes.get(process_id)
        if managed is None:
            return False

        proc = managed.proc
        if proc.poll() is not None:
            managed.returncode = proc.returncode
            return True

        if sys.platform == "win32":
            # Kill the full Windows process tree first while parent->child links are intact.
            subprocess.run(
                ["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                capture_output=True,
                check=False,
            )
            try:
                proc.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=1.0)
        else:
            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            except Exception:
                proc.terminate()
            try:
                proc.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                try:
                    pgid = os.getpgid(proc.pid)
                    os.killpg(pgid, signal.SIGKILL)
                except Exception:
                    proc.kill()
                proc.wait(timeout=1.0)

        if proc.stdout is not None:
            try:
                proc.stdout.close()
            except Exception:
                pass
        if managed.reader_thread is not None and managed.reader_thread.is_alive():
            managed.reader_thread.join(timeout=1.0)
        managed.returncode = proc.poll()
        return True

    def terminate_all(self) -> None:
        """Terminate all tracked subprocesses on server shutdown."""
        with self._lock:
            ids = list(self._processes.keys())
        for pid_key in ids:
            try:
                self.terminate(pid_key)
            except Exception:
                pass

    def list_pids(self) -> dict[str, int]:
        with self._lock:
            return {k: v.proc.pid for k, v in self._processes.items()}
