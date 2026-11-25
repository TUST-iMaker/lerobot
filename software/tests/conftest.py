"""Ensure workspace root (first) and software/ + software/tests/ are on sys.path for bare pytest runs."""

from __future__ import annotations

import sys
from pathlib import Path

_TESTS_DIR = Path(__file__).resolve().parent
_SOFTWARE_DIR = _TESTS_DIR.parent
_WORKSPACE_ROOT = _SOFTWARE_DIR.parent

_root_str = str(_WORKSPACE_ROOT)
if _root_str not in sys.path:
    sys.path.insert(0, _root_str)

# Lock top-level `tests` package in sys.modules before adding `software/` to sys.path
import tests  # noqa: E402, F401

for _p in (_SOFTWARE_DIR, _TESTS_DIR):
    _s = str(_p)
    if _s not in sys.path:
        sys.path.append(_s)
