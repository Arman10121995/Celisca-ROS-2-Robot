#!/usr/bin/env python3
"""Render the Mermaid diagrams in the docs to docs/diagrams/*.png (for the
GUI's documentation reader; GitHub renders the Mermaid itself).

    python3 scripts/render_doc_diagrams.py           # new/changed diagrams
    python3 scripts/render_doc_diagrams.py --force   # all of them
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'robot_lab_gui'))

from robot_lab_gui.doc_diagrams import render_all  # noqa: E402

used = render_all(ROOT, force='--force' in sys.argv)
print('%d diagrams in docs/diagrams' % len(used))
