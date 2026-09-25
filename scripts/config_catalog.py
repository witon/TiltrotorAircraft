#!/usr/bin/env python3
"""Load airframe configs from params/configs/<id>/config.json."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
PARAMS_DIR = REPO_ROOT / "params"
CONFIGS_DIR = PARAMS_DIR / "configs"
DEFAULT_CONFIG_ID = "bicopter"


@dataclass(frozen=True)
class AirframeConfig:
    config_id: str
    title: str
    dir: Path
    project_param: Path
    lua: Path | None
    lua_remote_name: str | None
    calib_required: tuple[str, ...]
    calib_optional: tuple[str, ...]

    @property
    def aircraft_dir(self) -> Path:
        return self.dir / "aircraft"


def _as_str_list(value, field: str, config_id: str) -> tuple[str, ...]:
    if value is None:
        return ()
    if not isinstance(value, list) or any(not isinstance(x, str) for x in value):
        raise ValueError(f"{config_id}: {field} must be a list of strings")
    return tuple(value)


def load_config(config_id: str) -> AirframeConfig:
    config_id = config_id.strip()
    if not config_id or "/" in config_id or "\\" in config_id:
        raise ValueError(f"invalid config id: {config_id!r}")
    cfg_dir = CONFIGS_DIR / config_id
    path = cfg_dir / "config.json"
    if not path.is_file():
        known = ", ".join(list_config_ids()) or "(none)"
        raise FileNotFoundError(
            f"config not found: {config_id} ({path})\nKnown: {known}"
        )
    raw = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"{config_id}: config.json must be an object")

    declared = raw.get("id", config_id)
    if declared != config_id:
        raise ValueError(f"{config_id}: id {declared!r} does not match directory")

    title = raw.get("title") or config_id
    if not isinstance(title, str):
        raise ValueError(f"{config_id}: title must be a string")

    project_name = raw.get("project_param", "project.param")
    if not isinstance(project_name, str):
        raise ValueError(f"{config_id}: project_param must be a string")
    project_param = cfg_dir / project_name
    if not project_param.is_file():
        raise FileNotFoundError(f"{config_id}: missing {project_param}")

    lua_rel = raw.get("lua")
    lua: Path | None = None
    lua_remote_name: str | None = None
    if lua_rel is not None:
        if not isinstance(lua_rel, str):
            raise ValueError(f"{config_id}: lua must be a string path or null")
        lua = (REPO_ROOT / lua_rel).resolve()
        if not lua.is_file():
            raise FileNotFoundError(f"{config_id}: lua not found: {lua}")
        lua_remote_name = lua.name

    calib = raw.get("calib") or {}
    if not isinstance(calib, dict):
        raise ValueError(f"{config_id}: calib must be an object")
    required = _as_str_list(calib.get("required"), "calib.required", config_id)
    optional = _as_str_list(calib.get("optional"), "calib.optional", config_id)

    return AirframeConfig(
        config_id=config_id,
        title=title,
        dir=cfg_dir,
        project_param=project_param,
        lua=lua,
        lua_remote_name=lua_remote_name,
        calib_required=required,
        calib_optional=optional,
    )


def list_config_ids() -> list[str]:
    if not CONFIGS_DIR.is_dir():
        return []
    ids: list[str] = []
    for child in sorted(CONFIGS_DIR.iterdir()):
        if child.is_dir() and (child / "config.json").is_file():
            ids.append(child.name)
    return ids


def list_configs() -> list[AirframeConfig]:
    return [load_config(cid) for cid in list_config_ids()]


def format_config_list() -> str:
    lines: list[str] = []
    for cfg in list_configs():
        lua_name = cfg.lua_remote_name or "(no lua)"
        lines.append(f"  {cfg.config_id:12} {cfg.title}  lua={lua_name}")
    return "\n".join(lines) if lines else "  (none)"


def other_lua_remote_names(selected: AirframeConfig) -> list[str]:
    names: list[str] = []
    seen: set[str] = set()
    for cfg in list_configs():
        name = cfg.lua_remote_name
        if not name or name == selected.lua_remote_name or name in seen:
            continue
        seen.add(name)
        names.append(name)
    return names
