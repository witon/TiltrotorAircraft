#!/usr/bin/env python3
"""Filter init.param using ArduPlane apm.pdef.json metadata tags.

Drops parameters marked Volatile, ReadOnly, or Calibration in official metadata.
"""

from __future__ import annotations

import json
import re
import urllib.request
from collections import Counter
from pathlib import Path

PARAMS_DIR = Path(__file__).resolve().parents[1] / "params"
META_URL = "https://autotest.ardupilot.org/Parameters/ArduPlane/apm.pdef.json"
META_PATH = PARAMS_DIR / "apm.pdef.json"
INIT_PATH = PARAMS_DIR / "init.param"
REMOVED_PATH = PARAMS_DIR / "init.param.removed.txt"
PARAM_LINE = re.compile(r"^([A-Za-z0-9_]+)\s*,\s*(-?[0-9.eE+-]+)\s*$")
SKIP_FLAGS = ("Volatile", "ReadOnly", "Calibration")


def truthy(v) -> bool:
    return str(v).strip().lower() in ("true", "1", "yes")


def load_meta() -> dict:
    if not META_PATH.is_file():
        print(f"Downloading {META_URL} ...")
        META_PATH.write_bytes(urllib.request.urlopen(META_URL, timeout=120).read())
    return json.loads(META_PATH.read_text(encoding="utf-8"))


def collect_flagged(meta: dict) -> dict[str, set[str]]:
    """Walk group -> param dictionaries; names in JSON are already full."""
    flagged: dict[str, set[str]] = {}

    def consider(name: str, fields: dict) -> None:
        hits = {f for f in SKIP_FLAGS if truthy(fields.get(f))}
        if hits:
            flagged[name] = hits

    def walk(obj: dict) -> None:
        for key, val in obj.items():
            if not isinstance(val, dict):
                continue
            # Parameter leaf: has description fields, children are not param dicts.
            has_desc = "DisplayName" in val or "Description" in val
            nested_params = [
                (k, v)
                for k, v in val.items()
                if isinstance(v, dict)
                and ("DisplayName" in v or "Description" in v)
            ]
            if nested_params:
                for name, fields in nested_params:
                    consider(name, fields)
                walk(val)
            elif has_desc:
                consider(key, val)
            else:
                walk(val)

    walk(meta)
    return flagged


def main() -> int:
    meta = load_meta()
    flagged = collect_flagged(meta)
    print(f"Metadata flagged params: {len(flagged)}")
    counts: Counter[str] = Counter()
    for fs in flagged.values():
        for f in fs:
            counts[f] += 1
    print("By flag:", dict(counts))

    lines = INIT_PATH.read_text(encoding="utf-8").splitlines(keepends=True)
    body: list[str] = []
    removed: list[tuple[str, str]] = []

    for raw in lines:
        stripped = raw.strip()
        if not stripped or stripped.startswith("#"):
            continue
        match = PARAM_LINE.match(stripped)
        if not match:
            body.append(raw if raw.endswith("\n") else raw + "\n")
            continue
        name = match.group(1)
        if name in flagged:
            removed.append((name, ",".join(sorted(flagged[name]))))
            continue
        body.append(raw if raw.endswith("\n") else raw + "\n")

    header = (
        "# Factory-default snapshot (reset FC export) with Q_ENABLE=1 for full upload.\n"
        "# Filtered via ArduPlane apm.pdef.json: dropped Volatile / ReadOnly / Calibration.\n"
        "# Regenerate filter: python scripts/filter-init-params.py\n"
        "# Full: upload this, reboot (Q_* appear), then matek-h743-mini-bicopter.param.\n"
        "# See docs/ardupilot-setup.md\n"
    )
    INIT_PATH.write_text(header + "".join(body), encoding="utf-8")
    REMOVED_PATH.write_text(
        "\n".join(f"{n}\t{f}" for n, f in removed) + ("\n" if removed else ""),
        encoding="utf-8",
    )

    kept_params = sum(1 for line in body if PARAM_LINE.match(line.strip()))
    print(f"Removed {len(removed)} -> {REMOVED_PATH.name}")
    print(f"Kept {kept_params} params in {INIT_PATH.name}")
    print("Sample removed:")
    for name, flags in removed[:25]:
        print(f"  {name} ({flags})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
