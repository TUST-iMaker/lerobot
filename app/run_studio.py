"""CLI launcher for TUST-iMaker Arm101 Studio."""

from __future__ import annotations

import argparse
import os
import sys
import threading
import webbrowser
from pathlib import Path

import uvicorn


def _ensure_workspace_on_syspath() -> Path:
    root = Path(__file__).resolve().parents[1]
    root_str = str(root)
    if root_str not in sys.path:
        sys.path.insert(0, root_str)
    return root


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="run_studio",
        description="Launch TUST-iMaker Arm101 Studio (FastAPI + Zero-Build Web UI).",
    )
    parser.add_argument(
        "--host",
        default=os.environ.get("STUDIO_HOST", "127.0.0.1"),
        help="Bind host (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("STUDIO_PORT", "8760")),
        help="Bind port (default: 8760)",
    )
    parser.add_argument(
        "--mode",
        choices=["auto", "real", "dry_run"],
        default=os.environ.get("STUDIO_MODE", "auto"),
        help="Operating mode: auto (probe and fallback to dry_run), real, or dry_run",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Force dry_run simulation mode (overrides --mode)",
    )
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="Do not automatically open the Studio URL in the default browser",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    _ensure_workspace_on_syspath()
    parser = build_parser()
    args = parser.parse_args(argv)

    effective_mode = "dry_run" if args.dry_run else args.mode
    os.environ["STUDIO_HOST"] = str(args.host)
    os.environ["STUDIO_PORT"] = str(args.port)
    os.environ["STUDIO_MODE"] = effective_mode

    url = f"http://{args.host}:{args.port}"
    print(f"[Arm101 Studio] Starting on {url} (requested_mode={effective_mode})")

    if not args.no_browser:
        timer = threading.Timer(0.9, lambda: webbrowser.open(url))
        timer.daemon = True
        timer.start()

    uvicorn.run(
        "app.backend.main:app",
        host=args.host,
        port=args.port,
        log_level="info",
        timeout_graceful_shutdown=2,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
