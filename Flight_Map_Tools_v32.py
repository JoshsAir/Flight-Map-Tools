#!/usr/bin/env python3
"""
FPV Flight Path Map (EdgeTX / ELRS telemetry logs)

What it does
------------
- Reads an EdgeTX CSV log
- Extracts lat/lon from the "GPS" column (format: "<lat> <lon>")
- Uses the "sats" column to control *track continuity*:
    * If sats < 5 -> the track breaks (discontinuous) until sats >= 5 again
    * If GPS is missing/blank/invalid -> the track breaks until GPS becomes valid again
- Removes consecutive duplicate GPS points (common at high telemetry rates)
- Writes interactive HTML map files next to the CSV using basemap tags in the filename
- Includes a default preset for quick export
- Lets you choose one or more basemap/tile styles before exporting
- Lets you choose which layer opens first in switchable-layer maps
- Optionally creates one HTML with all basemaps switchable inside the map
- Optionally adds a small Leaflet-attribution-style flight stats box
- Detects ArduPilot-like absolute altitude logs and reports relative altitude plus takeoff elevation
- Adds a single-CSV data-analysis heatmap mode for colouring a flight path by a selected CSV value
- Handles RSSI dBm from 1RSS/2RSS intelligently, ignoring zero-only second-chain values
- Optionally trims/hides the start and/or end of the displayed track for privacy
- Lets you preview/remove individual stats-box lines before export
- Adds a Dashware enrichment tab for computed telemetry columns and optional GPX output
- Supports user-saveable export presets stored beside the EXE
- Adds a dynamic metric scale bar to the map

Notes about internet
-------------------
- This program does NOT need internet to run or to create the HTML.
- The generated HTML uses Leaflet via CDN and the selected online tile/basemap provider,
  so viewing the map normally requires an internet connection.
- The default basemap avoids the direct tile.openstreetmap.org volunteer tile server,
  which can show 403r errors when local HTML files do not send Referer headers.

"""

from __future__ import annotations

import csv
import contextlib
import difflib
import io
import json
import math
import os
import re
import statistics
import struct
import sys
import uuid
import urllib.parse
import urllib.request
import zipfile
from dataclasses import dataclass
from datetime import datetime, timedelta
from html import escape
from typing import Any, Dict, List, Optional, Tuple

# ========= User-tweakable defaults =========
DEFAULT_ZOOM = 16
MIN_SATS = 5               # track is only drawn when sats >= this
DEDUP_DECIMALS = 6         # round lat/lon to this many decimals when deduping
HARD_POINT_LIMIT = 200_000 # safety: if a track is huge, it will be decimated

# Optional relaxed GPS-track mode. Five satellites remains the trusted/default threshold.
RELAXED_MIN_SATS = 4

# Flight-analysis defaults.
ANALYSIS_LOW_SATS_THRESHOLD = 5
ANALYSIS_MIN_TRACK_SATS = 4
ANALYSIS_SMOOTH_WINDOW_S = 5.0
ANALYSIS_COORD_SPEED_WINDOW_S = 2.0
ANALYSIS_INSPECTION_INTERVAL_S = 1.0

# Dashware heading-source selection. Prefer the original flight-controller heading
# when it has at least 97% moving-row coverage and disagrees with the centred
# 2-second GPS course by more than 30 degrees on no more than 3% of comparisons.
# Otherwise prefer the GPS-derived course when it has at least 97% moving coverage.
DASHWARE_HEADING_COMPARE_TOLERANCE_DEG = 30.0
DASHWARE_HEADING_MAX_BAD_PCT = 3.0
DASHWARE_HEADING_MIN_SPEED_KMH = 5.0

# Dashware/terrain enrichment. Local ArduPilot DAT/HGT terrain is the default and
# queries every distinct logged GPS coordinate. Optional OpenTopoData mode samples
# the path and interpolates between API results to respect public-service limits.
DASHWARE_TERRAIN_SAMPLE_DISTANCE_M = 30.0
DASHWARE_TERRAIN_SAMPLE_TIME_S = 3.0
DASHWARE_TERRAIN_BATCH_SIZE = 90
DASHWARE_TERRAIN_MAX_SAMPLES = 900

# Used for throttle percentage calculation from CH3(us).
THROTTLE_MIN_US = 988
THROTTLE_MAX_US = 2012

# Privacy mode default: hide this distance from the start and end of the displayed track.
DEFAULT_PRIVACY_METERS = 100.0

# Betaflight EdgeTX can briefly log takeoff elevation in Alt(m) before switching to relative altitude.
# These first MSL-looking samples are ignored for altitude stats only.
ALT_INITIAL_SCAN_LIMIT = 30
ALT_RELATIVE_ZERO_THRESHOLD_M = 10.0
ALT_MSL_LOOKING_THRESHOLD_M = 50.0
# Betaflight relative-altitude logs can occasionally glitch back to an MSL-looking value.
# A sudden, isolated jump larger than this is treated as a telemetry spike for altitude stats.
ALT_RELATIVE_SPIKE_JUMP_THRESHOLD_M = 250.0

# ArduPilot-like logs commonly keep Alt(m) as above-sea-level elevation for the entire log.
ARDUPILOT_ASL_MIN_ELEVATION_M = 50.0

# Basemap/tile choices shown in the menu.
# All options are internet-reliant when viewing the generated HTML map.
# The keys are the simple inputs the user can type.
# Detailed CyclOSM remains the manual default; the built-in quick preset opens with OpenStreetMap Standard.
TILE_PROVIDERS = {
    "1": {
        "name": "Detailed - CyclOSM (more labels/details; good for trails/paths)",
        "short": "detailed",
        "url": "https://{s}.tile-cyclosm.openstreetmap.fr/cyclosm/{z}/{x}/{y}.png",
        "options": {
            "attribution": '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors &copy; <a href="https://www.cyclosm.org">CyclOSM</a>',
            "subdomains": "abc",
            "maxNativeZoom": 18,
            "maxZoom": 20,
            "detectRetina": False,
        },
    },
    "2": {
        "name": "Default - CARTO Voyager (reliable, clean OSM-based map)",
        "short": "default",
        "url": "https://{s}.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}{r}.png",
        "options": {
            "attribution": '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors &copy; <a href="https://carto.com/attributions">CARTO</a>',
            "subdomains": "abcd",
            "maxNativeZoom": 19,
            "maxZoom": 20,
        },
    },
    "3": {
        "name": "OpenStreetMap Standard",
        "short": "osm",
        "url": "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
        "options": {
            "attribution": '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
            "maxNativeZoom": 19,
            "maxZoom": 20,
            "referrerPolicy": "strict-origin-when-cross-origin",
        },
    },
    "4": {
        "name": "Topographic - Esri World Topographic",
        "short": "topo",
        "url": "https://server.arcgisonline.com/ArcGIS/rest/services/World_Topo_Map/MapServer/tile/{z}/{y}/{x}",
        "options": {
            "attribution": 'Tiles &copy; Esri &mdash; Source: Esri, Garmin, USGS, NPS and other contributors',
            "maxNativeZoom": 19,
            "maxZoom": 20,
        },
    },
    "5": {
        "name": "Contours - OpenTopoMap (contours + hillshade)",
        "short": "contours",
        "url": "https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png",
        "options": {
            "attribution": 'Map data: &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors, <a href="https://viewfinderpanoramas.org">SRTM</a> | Map style: &copy; <a href="https://opentopomap.org">OpenTopoMap</a> (<a href="https://creativecommons.org/licenses/by-sa/3.0/">CC-BY-SA</a>)',
            "subdomains": "abc",
            "maxNativeZoom": 17,
            "maxZoom": 18,
        },
    },
    "6": {
        "name": "Satellite - Esri World Imagery",
        "short": "satellite",
        "url": "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        "options": {
            "attribution": 'Tiles &copy; Esri &mdash; Source: Esri, Maxar, Earthstar Geographics and other contributors',
            "maxNativeZoom": 18,
            "maxZoom": 20,
        },
    },
    "7": {
        "name": "Outdoors - Esri National Geographic World Map",
        "short": "natgeo",
        "url": "https://server.arcgisonline.com/ArcGIS/rest/services/NatGeo_World_Map/MapServer/tile/{z}/{y}/{x}",
        "options": {
            "attribution": 'Tiles &copy; Esri, National Geographic, Garmin, HERE, UNEP-WCMC, USGS, NASA, ESA, METI, NRCan, GEBCO, NOAA',
            "maxNativeZoom": 16,
            "maxZoom": 18,
        },
    },
}
DEFAULT_TILE_KEY = "1"
BUILTIN_PRESET_INITIAL_TILE_KEY = "3"  # OpenStreetMap Standard opens first in the built-in quick preset.
PRESETS_JSON_FILENAME = "fpv_flight_path_map_presets.json"
ALL_TILE_KEYS = sorted(TILE_PROVIDERS.keys(), key=int)
# =========================================

APP_VERSION_NUMBER = "v32"
MAPLE_LEAF_ICON_PNG_BASE64 = "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"
AIRCRAFT_GROUPS_JSON_FILENAME = "fpv_aircraft_grouping.json"


def _strip_quotes(s: str) -> str:
    s = s.strip()
    if (len(s) >= 2) and ((s[0] == s[-1]) and s[0] in ('"', "'")):
        return s[1:-1]
    return s


def normalize_path(user_input: str) -> str:
    """Cleans up quoted pasted paths and expands ~."""
    p = _strip_quotes(user_input.strip())
    p = os.path.expanduser(p)
    return os.path.abspath(p)


def parse_color(user_input: str) -> Optional[str]:
    """
    Accepts ROYGBIV colors by full name or single letter:
      r/red, o/orange, y/yellow, g/green, b/blue, i/indigo, v/violet/purple
    Also accepts a hex color like #ff00aa and common CSS color names.
    Returns a CSS color string, or None if not recognized.
    """
    s = (user_input or "").strip().lower()

    if not s:
        return "#3388ff"  # Leaflet default-ish

    roygbiv = {
        "r": "#ff0000", "red": "#ff0000",
        "o": "#ffa500", "orange": "#ffa500",
        "y": "#ffff00", "yellow": "#ffff00",
        "g": "#00aa00", "green": "#00aa00",
        "b": "#0000ff", "blue": "#0000ff",
        "i": "#4b0082", "indigo": "#4b0082",
        "v": "#8a2be2", "violet": "#8a2be2", "purple": "#8a2be2",
    }
    if s in roygbiv:
        return roygbiv[s]

    # Hex colors: #rgb or #rrggbb
    if s.startswith("#"):
        hexpart = s[1:]
        if len(hexpart) in (3, 6) and all(c in "0123456789abcdef" for c in hexpart):
            return s
        return None

    # Common CSS colors. This avoids silently accepting typos like "bluish".
    common_css = {
        "black", "white", "gray", "grey", "silver", "maroon", "red", "purple", "fuchsia",
        "green", "lime", "olive", "yellow", "navy", "blue", "teal", "aqua", "orange",
        "cyan", "magenta", "brown", "pink", "gold", "coral", "turquoise", "violet",
        "indigo", "transparent",
    }
    if s in common_css:
        return s

    return None


def choose_track_color() -> str:
    """Ask for a track colour and retry if the input is not recognized."""
    while True:
        raw = input("Track colour (ROYGBIV name/letter or #hex) [default blue]: ").strip()
        color = parse_color(raw)
        if color is not None:
            return color
        print("❌ Colour not recognized. Use ROYGBIV letters/words like b/blue/y/yellow, or a hex colour like #00aaff.")



def get_exe_folder() -> str:
    """Return the folder containing the EXE when frozen, or this .py file when run as a script."""
    if getattr(sys, "frozen", False):
        return os.path.dirname(os.path.abspath(sys.executable))
    return os.path.dirname(os.path.abspath(__file__))


def get_presets_json_path() -> str:
    """Presets live beside the EXE/script so they are easy to edit in Notepad."""
    return os.path.join(get_exe_folder(), PRESETS_JSON_FILENAME)


def get_aircraft_groups_json_path() -> str:
    """Aircraft grouping memory lives beside the EXE/script."""
    return os.path.join(get_exe_folder(), AIRCRAFT_GROUPS_JSON_FILENAME)


def load_aircraft_group_mapping() -> Dict[str, str]:
    """Load remembered raw aircraft/model name -> aircraft group mappings."""
    path = get_aircraft_groups_json_path()
    if not os.path.isfile(path):
        return {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        return {}
    raw_groups = data.get("groups", {}) if isinstance(data, dict) else {}
    if not isinstance(raw_groups, dict):
        return {}
    cleaned: Dict[str, str] = {}
    for raw_name, group_name in raw_groups.items():
        raw = str(raw_name).strip()
        group = str(group_name).strip()
        if raw and group:
            cleaned[raw] = group
    return cleaned


def save_aircraft_group_mapping(mapping: Dict[str, str], merge: bool = True) -> None:
    """Save remembered aircraft grouping mappings beside the EXE/script."""
    path = get_aircraft_groups_json_path()
    groups = load_aircraft_group_mapping() if merge else {}
    for raw_name, group_name in mapping.items():
        raw = str(raw_name).strip()
        group = str(group_name).strip()
        if raw and group:
            groups[raw] = group
    data = {
        "version": 1,
        "_instructions": [
            "Aircraft grouping memory for Flight Map Tools.",
            "The left side is the raw aircraft/model name detected from a CSV or filename.",
            "The right side is the final aircraft group name used in all-flights summaries.",
            "You can edit this file in Notepad, but it must remain valid JSON."
        ],
        "groups": dict(sorted(groups.items(), key=lambda item: item[0].lower())),
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)


def load_user_presets() -> List[Dict[str, Any]]:
    """Load saved user presets from JSON. Invalid presets are ignored safely."""
    path = get_presets_json_path()
    if not os.path.isfile(path):
        return []
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception as exc:
        print(f"⚠️  Could not read presets JSON: {exc}")
        return []

    presets = data.get("presets", []) if isinstance(data, dict) else []
    cleaned: List[Dict[str, Any]] = []
    for preset in presets:
        if not isinstance(preset, dict):
            continue
        name = str(preset.get("name", "")).strip()
        options = preset.get("options", {})
        if name and isinstance(options, dict):
            cleaned.append({"name": name, "options": options})
    return cleaned


def save_user_presets(presets: List[Dict[str, Any]]) -> None:
    """Write user presets while preserving shared parameter/unit settings."""
    path = get_presets_json_path()
    existing_settings: Dict[str, Any] = {}
    if os.path.isfile(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                existing = json.load(f)
            if isinstance(existing, dict) and isinstance(existing.get("parameter_settings"), dict):
                existing_settings = dict(existing["parameter_settings"])
        except Exception:
            existing_settings = {}
    data = {
        "version": 2,
        "_instructions": [
            "User presets and shared parameter settings for Flight Map Tools. You can edit this file in Notepad, but it must remain valid JSON.",
            "JSON does not allow real // comments, so these _instructions and _field_notes entries are safe comment-style notes that the script ignores.",
            "Each item in presets needs a name and an options object.",
            "parameter_settings stores Dashware unit/precision choices and saved per-parameter analysis thresholds."
        ],
        "_field_notes": {
            "presets": "List of saved map presets. Reorder entries to change their displayed order.",
            "name": "The preset name shown in the app.",
            "map_mode": "Type layers for one HTML with switchable basemaps, or separate for one HTML per selected basemap.",
            "tile_keys": "Basemap numbers to include for separate mode.",
            "initial_tile_key": "Basemap number that opens first in switchable-layer maps.",
            "stats_config.enabled": "true shows the stats box; false hides it.",
            "stats_config.groups": "Stats groups: default, signal, battery, throttle, gps, altitude.",
            "stats_config.position": "topright or topleft.",
            "stats_config.throttle_channel": "Throttle channel column such as CH3(us).",
            "privacy_config.enabled": "true removes private start/end GPS coordinates from generated HTML.",
            "privacy_config.start_meters": "Metres removed from the beginning of the visible GPS track.",
            "privacy_config.end_meters": "Metres removed from the end of the visible GPS track.",
            "min_sats": "5 is the normal trusted-GPS rule; 4 retains four-satellite route sections with warnings.",
            "parameter_settings.unit_system": "Metric, Imperial, or Custom.",
            "parameter_settings.elapsed_format": "Seconds, Decimal minutes, or Clock H:MM:SS.mmm.",
            "parameter_settings.*_unit": "Custom unit choices used only when unit_system is Custom.",
            "parameter_settings.*_decimals": "Number of decimal places written to added Dashware columns.",
            "parameter_settings.angular_rate_unit": "deg/s or rad/s for generated ground-track, roll, pitch, and yaw rates.",
            "parameter_settings.dashware_selected_fields": "List of Dashware column IDs restored when the app opens.",
            "parameter_settings.clamp_negative_agl": "true writes negative generated AGL terrain-model results as 0; original CSV fields remain unchanged.",
            "parameter_settings.terrain_source": "Local terrain files, OpenTopoData online, or Local first then online fallback.",
            "parameter_settings.terrain_folder": "Folder recursively scanned for ArduPilot .DAT and SRTM .HGT files.",
            "parameter_settings.joke_altitude_cap": "true caps generated altitude values at 400 ft / 122 m. Original CSV data is never changed.",
            "parameter_settings.analysis_profiles": "Saved analysis rule and threshold choices keyed by parameter ID.",
            "parameter_settings.analysis_png_width": "Standalone Plotly timeline PNG width in pixels (1920 by default).",
            "parameter_settings.analysis_png_height": "Standalone Plotly timeline PNG height in pixels (1080 by default).",
            "parameter_settings.analysis_chart_title": "Optional title drawn inside exported timeline PNG files.",
            "parameter_settings.analysis_png_filename": "Optional default filename used by the HTML timeline PNG download button."
        },
        "parameter_settings": existing_settings,
        "presets": presets,
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    print(f"Preset/settings file saved: {path}")


DEFAULT_PARAMETER_SETTINGS: Dict[str, Any] = {
    "unit_system": "Metric",
    "elapsed_format": "Seconds",
    "distance_unit": "m",
    "long_distance_unit": "km",
    "speed_unit": "km/h",
    "altitude_unit": "m",
    "vertical_speed_unit": "m/s",
    "acceleration_unit": "m/s²",
    "angular_rate_unit": "deg/s",
    "efficiency_distance_unit": "km",
    "short_decimals": 0,
    "long_decimals": 2,
    "altitude_decimals": 1,
    "speed_decimals": 1,
    "general_decimals": 2,
    "joke_altitude_cap": False,
    "clamp_negative_agl": True,
    "dashware_selected_fields": [],
    "terrain_source": "Local terrain files",
    "terrain_folder": "",
    "analysis_profiles": {},
    "analysis_png_width": 1920,
    "analysis_png_height": 1080,
    "analysis_chart_title": "",
    "analysis_png_filename": "",
}


def load_parameter_settings() -> Dict[str, Any]:
    """Load shared Dashware unit/precision settings from the existing preset JSON."""
    settings = dict(DEFAULT_PARAMETER_SETTINGS)
    path = get_presets_json_path()
    if not os.path.isfile(path):
        return settings
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        raw = data.get("parameter_settings", {}) if isinstance(data, dict) else {}
        if isinstance(raw, dict):
            settings.update(raw)
    except Exception:
        pass
    return settings


def save_parameter_settings(settings: Dict[str, Any]) -> None:
    """Save shared Dashware settings without removing existing map presets."""
    path = get_presets_json_path()
    presets = load_user_presets()
    current: Dict[str, Any] = {}
    if os.path.isfile(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                loaded = json.load(f)
            if isinstance(loaded, dict):
                current = loaded
        except Exception:
            current = {}
    current["version"] = max(2, int(current.get("version", 1) or 1))
    current.setdefault("_instructions", [
        "User presets and shared parameter settings for Flight Map Tools.",
        "This file must remain valid JSON."
    ])
    field_notes = current.setdefault("_field_notes", {})
    if isinstance(field_notes, dict):
        field_notes.update({
            "parameter_settings.unit_system": "Metric, Imperial, or Custom.",
            "parameter_settings.elapsed_format": "Seconds, Decimal minutes, or Clock H:MM:SS.mmm. Decimal minutes always writes two decimal places.",
            "parameter_settings.*_unit": "Custom unit choices used when unit_system is Custom.",
            "parameter_settings.*_decimals": "Number of decimal places written to the matching generated Dashware columns.",
            "parameter_settings.angular_rate_unit": "deg/s or rad/s for generated ground-track, roll, pitch, and yaw rates.",
            "parameter_settings.dashware_selected_fields": "Dashware column IDs restored as checked when the app opens.",
            "parameter_settings.clamp_negative_agl": "true writes negative generated AGL terrain-model results as 0; original CSV fields remain unchanged.",
            "parameter_settings.terrain_source": "Local terrain files, OpenTopoData online, or Local first then online fallback.",
            "parameter_settings.terrain_folder": "Folder recursively scanned for ArduPilot .DAT and SRTM .HGT files.",
            "parameter_settings.analysis_profiles": "Saved analysis rule and threshold choices keyed by parameter ID.",
            "parameter_settings.analysis_png_width": "Standalone Plotly timeline PNG width in pixels (1920 by default).",
            "parameter_settings.analysis_png_height": "Standalone Plotly timeline PNG height in pixels (1080 by default).",
            "parameter_settings.analysis_chart_title": "Optional title drawn inside exported timeline PNG files.",
            "parameter_settings.analysis_png_filename": "Optional default filename used by the HTML timeline PNG download button.",
        })
    current["presets"] = presets
    clean = dict(DEFAULT_PARAMETER_SETTINGS)
    existing_parameter_settings = current.get("parameter_settings", {})
    if isinstance(existing_parameter_settings, dict):
        clean.update(existing_parameter_settings)
    clean.update(settings or {})
    current["parameter_settings"] = clean
    with open(path, "w", encoding="utf-8") as f:
        json.dump(current, f, indent=2)

def _run_options_for_preset_storage(run_options: Dict[str, Any]) -> Dict[str, Any]:
    """Store export settings, but not the current track colour so colour can stay flight-specific."""
    stored = json.loads(json.dumps(run_options))  # simple deep copy of JSON-safe values
    stored.pop("color_css", None)
    return stored


def _sanitize_loaded_run_options(options: Dict[str, Any], color_css: str) -> Dict[str, Any]:
    """Make a loaded preset safe against renamed/removed tile providers or older JSON formats."""
    if not isinstance(options, dict):
        options = {}

    map_mode = str(options.get("map_mode", "layers")).lower()
    if map_mode not in ("layers", "separate"):
        map_mode = "layers"

    initial_tile_key = str(options.get("initial_tile_key", BUILTIN_PRESET_INITIAL_TILE_KEY))
    if initial_tile_key not in TILE_PROVIDERS:
        initial_tile_key = BUILTIN_PRESET_INITIAL_TILE_KEY

    raw_tile_keys = options.get("tile_keys", ALL_TILE_KEYS)
    if not isinstance(raw_tile_keys, list):
        raw_tile_keys = ALL_TILE_KEYS
    tile_keys = [str(k) for k in raw_tile_keys if str(k) in TILE_PROVIDERS]
    if not tile_keys:
        tile_keys = [DEFAULT_TILE_KEY]
    if map_mode == "layers":
        tile_keys = _ordered_tile_keys(initial_tile_key, ALL_TILE_KEYS)

    stats_config = options.get("stats_config", {"enabled": False})
    if not isinstance(stats_config, dict):
        stats_config = {"enabled": False}
    stats_config = dict(stats_config)
    if "groups" in stats_config and isinstance(stats_config["groups"], list):
        valid_group_keys = {v["key"] for v in STATS_GROUPS.values()}
        stats_config["groups"] = [g for g in stats_config["groups"] if g in valid_group_keys]
    if stats_config.get("enabled") and not stats_config.get("groups"):
        stats_config["groups"] = ["default"]
    if stats_config.get("position") not in ("topright", "topleft"):
        stats_config["position"] = "topright"
    stats_config.setdefault("throttle_channel", "CH3(us)")

    privacy_config = options.get("privacy_config", {"enabled": False})
    if not isinstance(privacy_config, dict):
        privacy_config = {"enabled": False}
    privacy_config = dict(privacy_config)
    privacy_config.setdefault("enabled", False)
    privacy_config.setdefault("meters", 0.0)
    privacy_config.setdefault("start_meters", privacy_config.get("meters", 0.0))
    privacy_config.setdefault("end_meters", privacy_config.get("meters", 0.0))
    privacy_config.setdefault("show_status", False)

    try:
        min_sats = int(options.get("min_sats", MIN_SATS))
    except Exception:
        min_sats = MIN_SATS
    min_sats = RELAXED_MIN_SATS if min_sats <= RELAXED_MIN_SATS else MIN_SATS

    return {
        "color_css": color_css,
        "map_mode": map_mode,
        "tile_keys": tile_keys,
        "initial_tile_key": initial_tile_key,
        "stats_config": stats_config,
        "privacy_config": privacy_config,
        "min_sats": min_sats,
    }


def maybe_save_or_overwrite_preset(run_options: Dict[str, Any]) -> None:
    """Ask whether to save/overwrite a user preset after custom choices are made."""
    presets = load_user_presets()
    print("\nSave these settings as a user preset?")
    print("Press Enter or type n = do not save.")
    if presets:
        print("Type an existing preset number to overwrite it, or type a new preset name.")
        for i, preset in enumerate(presets, start=1):
            print(f"{i}) {preset['name']}")
    else:
        print("Type a preset name to save it, such as mountain flight or prairie flight.")

    raw = input("Save/overwrite preset: ").strip()
    if not raw or raw.lower() in ("n", "no", "none", "skip", "off"):
        return

    stored_options = _run_options_for_preset_storage(run_options)

    if raw.isdigit() and presets:
        idx = int(raw)
        if 1 <= idx <= len(presets):
            old_name = presets[idx - 1]["name"]
            if _ask_yes_no(f"Overwrite preset '{old_name}'?", default=True):
                presets[idx - 1] = {"name": old_name, "options": stored_options}
                save_user_presets(presets)
            return
        print("❌ Preset number not recognized, so nothing was saved.")
        return

    name = raw.strip()
    if not name:
        return

    for i, preset in enumerate(presets):
        if preset["name"].strip().lower() == name.lower():
            if _ask_yes_no(f"Preset '{preset['name']}' already exists. Overwrite it?", default=True):
                presets[i] = {"name": preset["name"], "options": stored_options}
                save_user_presets(presets)
            return

    presets.append({"name": name, "options": stored_options})
    save_user_presets(presets)


def _parse_tile_choices(user_input: str) -> Optional[List[str]]:
    """
    Parse tile choices from inputs like:
      Enter -> default
      1 -> option 1
      14 -> options 1 and 4
      1234567 -> all options
      1,4 or 1 4 -> options 1 and 4
    Returns None if invalid.
    """
    s = (user_input or "").strip().lower()

    if not s:
        return [DEFAULT_TILE_KEY]

    if s in ("all", "a", "*"):
        keys = ALL_TILE_KEYS
    else:
        # Allow compact input like 14, or separated input like 1,4 / 1 4.
        cleaned = s.replace(",", "").replace(" ", "").replace(";", "").replace("-", "")
        if not cleaned or any(ch not in TILE_PROVIDERS for ch in cleaned):
            return None
        keys = list(cleaned)

    # Deduplicate while preserving order.
    seen = set()
    selected: List[str] = []
    for key in keys:
        if key not in seen:
            selected.append(key)
            seen.add(key)

    return selected if selected else None


def choose_map_output_mode() -> str:
    """
    Choose how basemaps are handled:
      separate -> one HTML per selected basemap
      layers   -> one HTML with all basemaps switchable inside the map
    """
    print("\nBasemap output mode:")
    print("1) Separate HTML file(s), one per selected basemap")
    print("2) One HTML file with all basemaps switchable inside the map [default]")
    raw = input("Choose mode (press Enter for 2): ").strip().lower()

    if raw in ("", "2", "layers", "layer", "l", "switch", "switchable"):
        return "layers"
    if raw in ("1", "separate", "s"):
        return "separate"

    while raw not in ("", "1", "2", "separate", "s", "layers", "layer", "l", "switch", "switchable"):
        print("❌ Invalid choice. Enter 1 for separate files or 2 for one switchable-layer map.")
        raw = input("Choose mode (press Enter for 2): ").strip().lower()
        if raw in ("", "2", "layers", "layer", "l", "switch", "switchable"):
            return "layers"
        if raw in ("1", "separate", "s"):
            return "separate"

    return "layers"


def choose_tile_providers() -> List[str]:
    """Ask which basemap/tile providers to use. Enter = default. Multiple choices allowed."""
    print("\nChoose basemap / map tiles:")
    for key in ALL_TILE_KEYS:
        default_note = " [default]" if key == DEFAULT_TILE_KEY else ""
        short_name = TILE_PROVIDERS[key]["short"]
        print(f"{key}) {TILE_PROVIDERS[key]['name']} -> filename tag ({short_name}){default_note}")

    print("You can choose more than one. Examples: 14 = options 1 and 4, 1234567 = all options.")
    raw = input(f"Basemap choice(s) (press Enter for {DEFAULT_TILE_KEY}): ").strip()
    choices = _parse_tile_choices(raw)

    while choices is None:
        print("❌ Invalid basemap choice. Type numbers from the list, like 1, 14, or 1234567.")
        raw = input(f"Basemap choice(s) (press Enter for {DEFAULT_TILE_KEY}): ").strip()
        choices = _parse_tile_choices(raw)

    print("Using basemap(s):")
    for key in choices:
        print(f"   {key}) {TILE_PROVIDERS[key]['name']}")
    return choices


def _ordered_tile_keys(initial_tile_key: str, tile_keys: Optional[List[str]] = None) -> List[str]:
    """Return tile keys with initial_tile_key first, then remaining keys in their normal order."""
    keys = list(tile_keys if tile_keys else ALL_TILE_KEYS)
    if initial_tile_key not in TILE_PROVIDERS:
        initial_tile_key = DEFAULT_TILE_KEY

    ordered: List[str] = []
    if initial_tile_key in keys:
        ordered.append(initial_tile_key)
    else:
        ordered.append(initial_tile_key)

    for key in keys:
        if key not in ordered:
            ordered.append(key)

    return ordered


def choose_initial_tile_provider(default_key: str = DEFAULT_TILE_KEY) -> str:
    """Ask which basemap should open first in switchable-layer maps."""
    if default_key not in TILE_PROVIDERS:
        default_key = DEFAULT_TILE_KEY

    while True:
        print("\nChoose the basemap that should open first in switchable-layer maps:")
        for key in ALL_TILE_KEYS:
            default_note = " [default]" if key == default_key else ""
            print(f"{key}) {TILE_PROVIDERS[key]['name']}{default_note}")

        raw = input(f"Opening/default layer (press Enter for {default_key}): ").strip().lower()
        if not raw:
            return default_key

        if raw in TILE_PROVIDERS:
            return raw

        for key, provider in TILE_PROVIDERS.items():
            if raw == provider.get("short", "").lower():
                return key

        print(f"❌ Layer choice not recognized. Accepted inputs: Enter or one of {', '.join(ALL_TILE_KEYS)}.")


STATS_GROUPS = {
    "1": {
        "key": "default",
        "name": "Basic flight stats (date, times, air time, avg/max speed, max distance, total distance, avg RSNR)",
    },
    "2": {
        "key": "signal",
        "name": "Signal/link stats (RQly, RSNR, RSSI dBm, TPWR)",
    },
    "3": {
        "key": "battery",
        "name": "Battery/power stats (current, power, capacity used, efficiency)",
    },
    "4": {
        "key": "throttle",
        "name": "Throttle stats from a selectable CH#(us) channel",
    },
    "5": {
        "key": "gps",
        "name": "GPS quality stats (average sats and max sats)",
    },
    "6": {
        "key": "altitude",
        "name": "Relative altitude stats with time to max altitude",
    },
    "7": {
        "key": "agl_altitude",
        "name": "Altitude above ground level (requires terrain data)",
    },
    "8": {
        "key": "extra_telemetry",
        "name": "Additional telemetry sensors when present (logged VSpd / temperature)",
    },
}


def choose_flight_stats() -> Dict[str, Any]:
    """
    Ask whether to add a flight-stats box and which stats to include.

    Enter/default means ALL stats. Option 1 is the smaller Basic flight stats group.
    Returns:
      {"enabled": bool, "groups": [group keys], "position": "topright" or "topleft"}
    """
    while True:
        print("\nAdd flight stats box to the map?")
        print("Enter/all/default = include all available stat groups.")
        print("n/no/none = no flight stats.")
        for num in sorted(STATS_GROUPS.keys(), key=int):
            print(f"{num}) {STATS_GROUPS[num]['name']}")
        raw = input("Stats choice (Enter = all, n, or numbers like 124): ").strip().lower()
        if raw in ("n", "no", "none", "off", "0"):
            return {"enabled": False, "groups": [], "position": "topright"}
        if raw in ("", "all", "a", "*", "default", "d"):
            groups = [STATS_GROUPS[num]["key"] for num in sorted(STATS_GROUPS.keys(), key=int)]
            break
        cleaned = raw.replace(",", "").replace(" ", "").replace(";", "").replace("-", "")
        if cleaned and all(ch in STATS_GROUPS for ch in cleaned):
            groups = []
            seen = set()
            for ch in cleaned:
                group_key = STATS_GROUPS[ch]["key"]
                if group_key not in seen:
                    groups.append(group_key)
                    seen.add(group_key)
            break
        print("❌ Invalid stats choice. Accepted inputs: Enter/all/default, n/no, or numbers like 124.")
    position = choose_stats_position()
    return {"enabled": True, "groups": groups, "position": position}


def choose_stats_position() -> str:
    """Ask where to place the stats box. Only top right and top left are supported."""
    while True:
        print("\nStats box location:")
        print("1) Top right [default]")
        print("2) Top left")
        raw = input("Choose location (press Enter for top right): ").strip().lower()

        if raw in ("", "1", "tr", "topright", "top right", "right"):
            return "topright"
        if raw in ("2", "tl", "topleft", "top left", "left"):
            return "topleft"

        print("❌ Location not recognized. Accepted inputs: Enter/1/tr/topright or 2/tl/topleft.")


def _ask_yes_no(prompt: str, default: bool = False) -> bool:
    """Strict yes/no prompt. Returns default on Enter."""
    default_text = "Y/n" if default else "y/N"
    while True:
        raw = input(f"{prompt} [{default_text}]: ").strip().lower()
        if not raw:
            return default
        if raw in ("y", "yes", "true", "1", "on"):
            return True
        if raw in ("n", "no", "false", "0", "off"):
            return False
        print("❌ Input not recognized. Accepted inputs: y/yes or n/no.")


def choose_privacy_mode(stats_enabled: bool = False) -> Dict[str, Any]:
    """
    Privacy mode removes GPS points near the start and/or end of the DISPLAYED map.
    It does not change the original CSV, and removed coordinates are not written into the HTML.
    """
    while True:
        print("\nPrivacy mode:")
        print("1) Off [default]")
        print(f"2) On with default blanket trim ({DEFAULT_PRIVACY_METERS:.0f} m from start and end)")
        print("3) Custom blanket trim: same metres from start and end")
        print("4) Fine-tune start/end separately")
        print("Or type a number to use that many metres from both start and end.")
        raw = input("Privacy choice (Enter = off): ").strip().lower()

        if raw in ("", "1", "n", "no", "none", "off", "0"):
            return {
                "enabled": False,
                "meters": 0.0,
                "start_meters": 0.0,
                "end_meters": 0.0,
                "show_status": False,
            }

        if raw in ("2", "y", "yes", "on", "default", "d"):
            config = {
                "enabled": True,
                "meters": DEFAULT_PRIVACY_METERS,
                "start_meters": DEFAULT_PRIVACY_METERS,
                "end_meters": DEFAULT_PRIVACY_METERS,
                "show_status": False,
            }
            break

        if raw in ("3", "same", "blanket", "b"):
            meters = _ask_float("Blanket trim distance in metres", min_value=0.0)
            if meters <= 0:
                return {
                    "enabled": False,
                    "meters": 0.0,
                    "start_meters": 0.0,
                    "end_meters": 0.0,
                    "show_status": False,
                }
            config = {
                "enabled": True,
                "meters": meters,
                "start_meters": meters,
                "end_meters": meters,
                "show_status": False,
            }
            break

        if raw in ("4", "fine", "finetune", "separate", "s"):
            start_m = _ask_float("Trim distance from START in metres", min_value=0.0)
            end_m = _ask_float("Trim distance from END in metres", min_value=0.0)
            if start_m <= 0 and end_m <= 0:
                return {
                    "enabled": False,
                    "meters": 0.0,
                    "start_meters": 0.0,
                    "end_meters": 0.0,
                    "show_status": False,
                }
            config = {
                "enabled": True,
                "meters": max(start_m, end_m),
                "start_meters": start_m,
                "end_meters": end_m,
                "show_status": False,
            }
            break

        try:
            meters = float(raw)
            if meters <= 0:
                return {
                    "enabled": False,
                    "meters": 0.0,
                    "start_meters": 0.0,
                    "end_meters": 0.0,
                    "show_status": False,
                }
            config = {
                "enabled": True,
                "meters": meters,
                "start_meters": meters,
                "end_meters": meters,
                "show_status": False,
            }
            break
        except Exception:
            print("❌ Privacy input not recognized. Accepted: Enter/1/off, 2/default, 3/custom blanket, 4/separate, or a metre value like 150.")

    if stats_enabled:
        config["show_status"] = _ask_yes_no("Show privacy status in the stats box?", default=False)

    return config


def _ask_float(prompt: str, min_value: Optional[float] = None) -> float:
    """Strict float prompt with optional minimum value."""
    while True:
        raw = input(f"{prompt}: ").strip()
        try:
            value = float(raw)
            if min_value is not None and value < min_value:
                print(f"❌ Please enter a number greater than or equal to {min_value}.")
                continue
            return value
        except Exception:
            print("❌ Please enter a number, like 0, 50, 100, or 1000.")


def get_csv_header(csv_path: Optional[str]) -> List[str]:
    """Return the header row from a CSV file, or [] if it cannot be read."""
    if not csv_path:
        return []
    try:
        with open(csv_path, "r", encoding="utf-8-sig", errors="replace", newline="") as f:
            dialect = sniff_dialect(f)
            reader = csv.reader(f, dialect)
            return next(reader)
    except Exception:
        return []


def available_channel_columns_from_header(header: List[str]) -> List[str]:
    """Return CH#(us) columns sorted numerically, like CH1(us), CH2(us), ..."""
    channels: List[Tuple[int, str]] = []
    for col_name in header:
        name = col_name.strip()
        m = re_match_channel(name)
        if m is not None:
            channels.append((m, name))
    channels.sort(key=lambda x: x[0])
    return [name for _, name in channels]


def re_match_channel(name: str) -> Optional[int]:
    """Parse a channel column name and return its channel number if it looks like CH# or CH#(us)."""
    if not name.lower().startswith("ch"):
        return None
    digits = ""
    for ch in name[2:]:
        if ch.isdigit():
            digits += ch
        else:
            break
    if not digits:
        return None
    try:
        return int(digits)
    except Exception:
        return None


def choose_throttle_channel_if_needed(stats_config: Dict[str, Any], sample_csv_path: Optional[str]) -> str:
    """If throttle stats were selected, ask which CH#(us) column is throttle. Default is CH3(us)."""
    if not stats_config.get("enabled") or "throttle" not in stats_config.get("groups", []):
        return "CH3(us)"

    header = get_csv_header(sample_csv_path)
    channels = available_channel_columns_from_header(header)

    print("\nThrottle channel for throttle stats:")
    if not channels:
        print("No CH#(us) columns were detected from the sample CSV, so throttle stats will use CH3(us).")
        return "CH3(us)"

    while True:
        for i, channel_name in enumerate(channels, start=1):
            default_note = " [default]" if re_match_channel(channel_name) == 3 else ""
            print(f"{i}) {channel_name}{default_note}")

        raw = input("Choose throttle channel (press Enter for CH3, or type list number / CH number): ").strip().lower()
        if raw in ("", "default", "d"):
            return "CH3(us)"

        try:
            list_index = int(raw)
            if 1 <= list_index <= len(channels):
                return channels[list_index - 1]
        except Exception:
            pass

        requested = raw if raw.startswith("ch") else f"ch{raw}"
        for channel_name in channels:
            if channel_name.lower().startswith(requested):
                return channel_name

        print("❌ Throttle channel not recognized. Accepted inputs: Enter for CH3, a list number, or a channel like CH1/CH2/CH3.")


def detect_altitude_source_from_values(values: List[float], flight_stack: str = "unknown") -> str:
    """Classify the *meaning* of Alt(m), separately from firmware identification.

    ArduPilot can legitimately arrive in EdgeTX as either MSL/ASL altitude or relative
    altitude, so firmware is no longer inferred from altitude alone.  Betaflight may also
    emit one or more initial MSL-looking values before switching to relative altitude.
    """
    vals = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    if len(vals) < 2:
        return "unknown"
    first = vals[0]
    has_near_zero = any(abs(v) <= ALT_RELATIVE_ZERO_THRESHOLD_M for v in vals)
    asl_like = abs(first) >= ARDUPILOT_ASL_MIN_ELEVATION_M and not has_near_zero

    stack = str(flight_stack or "unknown").lower()
    if stack == "ardupilot":
        return "ardupilot_asl" if asl_like else "ardupilot_relative"
    if stack == "inav":
        return "inav_asl" if asl_like else "inav_relative"
    if stack == "betaflight":
        return "betaflight_relative"
    if asl_like:
        return "asl_unknown"
    return "relative_unknown"


def _altitude_source_is_asl(source: Any) -> bool:
    return str(source or "") in ("ardupilot_asl", "inav_asl", "asl_unknown")


def detect_altitude_source_csv(csv_path: Optional[str]) -> str:
    """Inspect firmware cues and Alt(m) semantics without depending on CSV column order."""
    if not csv_path:
        return "unknown"
    try:
        with open(csv_path, "r", encoding="utf-8-sig", errors="replace", newline="") as f:
            dialect = sniff_dialect(f)
            reader = csv.reader(f, dialect)
            header = next(reader, None)
            if not header:
                return "unknown"
            rows = [list(row) for row in reader]
        alt_idx = _best_numeric_col_index(header, rows, ["Alt(m)", "Alt", "alt (m)", "altm"])
        if alt_idx is None:
            return "unknown"
        values = [_parse_float(_clean_cell(row, alt_idx)) for row in rows[: max(ALT_INITIAL_SCAN_LIMIT, 3000)]]
        stack = _detect_flight_stack_from_table(header, rows).get("stack", "unknown")
        return detect_altitude_source_from_values([v for v in values if v is not None], str(stack))
    except Exception:
        return "unknown"


def configure_ardupilot_throttle_handling(
    stats_config: Dict[str, Any],
    sample_csv_path: Optional[str],
    csv_paths: Optional[List[str]] = None,
) -> Dict[str, Any]:
    """Offer throttle removal only where the log actually shows significant controller-managed throttle time.

    v31 treated ArduPilot ASL altitude as a proxy for firmware.  v32 instead reads flight
    mode strings and uses a 10% logged-time threshold.  This also catches modern ArduPilot
    logs whose Alt(m) starts at zero/relative altitude while avoiding warnings on ordinary
    manual/FBWA-only logs.
    """
    if not stats_config.get("enabled") or "throttle" not in stats_config.get("groups", []):
        return stats_config

    paths = [p for p in (csv_paths or ([sample_csv_path] if sample_csv_path else [])) if p]
    flagged: List[Tuple[str, Dict[str, Any]]] = []
    for path in paths:
        assessment = assess_flight_autonomy_csv(path, str(stats_config.get("throttle_channel", "CH3(us)")))
        if assessment.get("classification") == "semi_autonomous":
            flagged.append((path, assessment))
    if not flagged:
        return stats_config

    stats_config = dict(stats_config)
    if len(paths) > 1:
        print(f"\nSemi-autonomous/controller-managed throttle detected in {len(flagged)} of {len(paths)} CSV file(s).")
        for path, assessment in flagged[:6]:
            modes = ", ".join(assessment.get("autonomous_modes", [])) or "controller-managed mode"
            print(f"  {os.path.basename(path)}: {assessment.get('autonomous_fraction', 0.0) * 100.0:.1f}% ({modes})")
        if len(flagged) > 6:
            print(f"  ...and {len(flagged) - 6} more file(s).")
        print("In these logs, the RC throttle channel is a command/input and may not equal actual motor/TECS output.")
        remove_for_ardu = _ask_yes_no("Remove throttle stats only from those semi-autonomous outputs in this batch?", default=True)
        stats_config["remove_throttle_for_ardupilot_logs"] = bool(remove_for_ardu)  # kept for preset backward compatibility
        if remove_for_ardu:
            print("Throttle stats will be removed only from semi-autonomous files; other files keep throttle stats.")
        else:
            print("Throttle stats will be kept for all files, with the interpretation warning above.")
        return stats_config

    path, assessment = flagged[0]
    modes = ", ".join(assessment.get("autonomous_modes", [])) or "controller-managed mode"
    print(f"\nSemi-autonomous/controller-managed throttle detected: {assessment.get('autonomous_fraction', 0.0) * 100.0:.1f}% of logged time ({modes}).")
    print("The RC throttle channel may not equal actual motor/TECS output during those periods.")
    remove = _ask_yes_no("Remove throttle stats from this output?", default=True)
    if remove:
        stats_config["groups"] = [g for g in stats_config.get("groups", []) if g != "throttle"]
        print("Throttle stats removed.")
    else:
        print("Throttle stats kept.")
    return stats_config

def maybe_remove_throttle_for_ardupilot(stats_config: Dict[str, Any], sample_csv_path: Optional[str]) -> Dict[str, Any]:
    """Backwards-compatible wrapper for single-file ArduPilot throttle handling."""
    return configure_ardupilot_throttle_handling(stats_config, sample_csv_path, [sample_csv_path] if sample_csv_path else [])


def effective_stats_config_for_csv(stats_config: Dict[str, Any], csv_path: Optional[str]) -> Dict[str, Any]:
    """Apply per-file stat adjustments, such as removing throttle only for semi-autonomous logs."""
    effective = dict(stats_config or {})
    groups = list(effective.get("groups", []))
    if effective.get("remove_throttle_for_ardupilot_logs") and csv_path:
        assessment = assess_flight_autonomy_csv(csv_path, str(effective.get("throttle_channel", "CH3(us)")))
        if assessment.get("classification") == "semi_autonomous":
            groups = [g for g in groups if g != "throttle"]
    effective["groups"] = groups
    return effective


def default_preset_options(color_css: str, sample_csv_path: Optional[str] = None) -> Dict[str, Any]:
    """Quick preset: one switchable-layer HTML, all stats at top right, privacy off."""
    all_stat_groups = [STATS_GROUPS[num]["key"] for num in sorted(STATS_GROUPS.keys(), key=int)]
    stats_config = {"enabled": True, "groups": all_stat_groups, "position": "topright", "throttle_channel": "CH3(us)"}
    return {
        "color_css": color_css,
        "map_mode": "layers",
        "tile_keys": _ordered_tile_keys(BUILTIN_PRESET_INITIAL_TILE_KEY, ALL_TILE_KEYS),
        "initial_tile_key": BUILTIN_PRESET_INITIAL_TILE_KEY,
        "stats_config": stats_config,
        "privacy_config": {"enabled": False, "meters": 0.0, "start_meters": 0.0, "end_meters": 0.0, "show_status": False},
        "min_sats": MIN_SATS,
    }


def choose_run_options(sample_csv_path: Optional[str] = None, preview_csv_paths: Optional[List[str]] = None) -> Dict[str, Any]:
    """Prompts that should be asked once and applied to one file or an entire recursive batch."""
    color_css = choose_track_color()
    preview_csv_paths = preview_csv_paths or ([sample_csv_path] if sample_csv_path else [])
    user_presets = load_user_presets()
    while True:
        print("\nDefault preset?")
        print("Press Enter = use built-in preset: one switchable-layer HTML, all flight stats, stats box top right, privacy off, OpenStreetMap Standard opens first.")
        print("Type n/no = customize all options manually.")
        if user_presets:
            print("Select custom user preset:")
            for i, preset in enumerate(user_presets, start=1):
                print(f"{i}) {preset['name']}")
            preset_raw = input("Use preset? [Enter=built-in preset, n=customize, or preset number]: ").strip().lower()
        else:
            preset_raw = input("Use preset? [Enter=built-in preset, n=customize]: ").strip().lower()
        if preset_raw in ("", "y", "yes", "preset", "default", "d"):
            print("Using built-in default preset.")
            run_options = default_preset_options(color_css, sample_csv_path)
            run_options["stats_config"] = configure_ardupilot_throttle_handling(run_options.get("stats_config", {}), sample_csv_path, preview_csv_paths)
            return finalize_run_options(run_options, sample_csv_path, preview_csv_paths)
        if user_presets and preset_raw.isdigit():
            idx = int(preset_raw)
            if 1 <= idx <= len(user_presets):
                preset = user_presets[idx - 1]
                print(f"Using user preset: {preset['name']}")
                run_options = _sanitize_loaded_run_options(preset.get("options", {}), color_css)
                run_options["stats_config"] = configure_ardupilot_throttle_handling(run_options.get("stats_config", {}), sample_csv_path, preview_csv_paths)
                return finalize_run_options(run_options, sample_csv_path, preview_csv_paths)
            print(f"❌ Preset number not recognized. Choose a number from 1 to {len(user_presets)}, press Enter, or type n.")
            continue
        if preset_raw in ("n", "no", "custom", "customize", "customise", "manual", "m"):
            break
        if user_presets:
            print("❌ Preset input not recognized. Accepted inputs: Enter/built-in preset, n/no/custom, or a listed preset number.")
        else:
            print("❌ Preset input not recognized. Accepted inputs: Enter/built-in preset or n/no/custom.")
    map_mode = choose_map_output_mode()
    initial_tile_key = DEFAULT_TILE_KEY
    if map_mode == "layers":
        initial_tile_key = choose_initial_tile_provider(DEFAULT_TILE_KEY)
        tile_keys = _ordered_tile_keys(initial_tile_key, ALL_TILE_KEYS)
        print(f"Using all basemaps inside one switchable-layer HTML. {TILE_PROVIDERS[initial_tile_key]['name']} loads first.")
    else:
        tile_keys = choose_tile_providers()
    stats_config = choose_flight_stats()
    stats_config = configure_ardupilot_throttle_handling(stats_config, sample_csv_path, preview_csv_paths)
    stats_config["throttle_channel"] = choose_throttle_channel_if_needed(stats_config, sample_csv_path)
    privacy_config = choose_privacy_mode(stats_enabled=bool(stats_config.get("enabled")))
    include_four_sat = _ask_yes_no(
        "Include GPS track rows with exactly 4 satellites? This may preserve more track, but GPS altitude/position can be less reliable",
        default=False,
    )
    run_options = {
        "color_css": color_css,
        "map_mode": map_mode,
        "tile_keys": tile_keys,
        "initial_tile_key": initial_tile_key,
        "stats_config": stats_config,
        "privacy_config": privacy_config,
        "min_sats": RELAXED_MIN_SATS if include_four_sat else MIN_SATS,
    }
    run_options = finalize_run_options(run_options, sample_csv_path, preview_csv_paths)
    maybe_save_or_overwrite_preset(run_options)
    return run_options


def sniff_dialect(f) -> csv.Dialect:
    """Try to detect delimiter; fall back to default CSV dialect."""
    pos = f.tell()
    sample = f.read(4096)
    f.seek(pos)
    try:
        return csv.Sniffer().sniff(sample, delimiters=[",", ";", "\t"])
    except Exception:
        return csv.excel


def _find_col_index(header: List[str], target: str) -> Optional[int]:
    """
    Finds a column index by case-insensitive exact match first, then "starts with".
    Returns None if not found.
    """
    header_norm = [h.strip().strip("\ufeff") for h in header]
    t = target.strip().lower()

    for i, h in enumerate(header_norm):
        if h.strip().lower() == t:
            return i

    for i, h in enumerate(header_norm):
        if h.strip().lower().startswith(t):
            return i

    return None


def _find_any_col_index(header: List[str], targets: List[str]) -> Optional[int]:
    """Try multiple target column names using _find_col_index."""
    for target in targets:
        idx = _find_col_index(header, target)
        if idx is not None:
            return idx
    return None


def _parse_sats(value: str) -> Optional[float]:
    """Parse sats; returns None if missing/invalid."""
    return _parse_float(value)


def _parse_float(value: Any) -> Optional[float]:
    """Parse a number from a CSV cell. Returns None if missing/invalid."""
    if value is None:
        return None
    s = str(value).strip()
    if not s:
        return None
    try:
        return float(s)
    except Exception:
        return None


def _clean_cell(row: List[str], idx: Optional[int]) -> str:
    if idx is None or idx >= len(row):
        return ""
    return (row[idx] or "").strip()


def _normalise_header_name(value: Any) -> str:
    """Normalise a CSV header for semantic matching without depending on column order."""
    return re.sub(r"\s+", "", str(value or "").strip().strip("\ufeff").lower())


def _matching_col_indices(header: List[str], targets: List[str], startswith: bool = False) -> List[int]:
    """Return all matching column indices, preserving source-column order."""
    wanted = [_normalise_header_name(t) for t in targets if str(t or "").strip()]
    matches: List[int] = []
    for i, name in enumerate(header):
        h = _normalise_header_name(name)
        if any((h.startswith(t) if startswith else h == t) for t in wanted):
            matches.append(i)
    return matches


def _numeric_column_profile(rows: List[List[str]], idx: int, sample_limit: int = 5000) -> Dict[str, float]:
    """Score a numeric telemetry column by coverage and useful variation.

    This is mainly used to resolve duplicate EdgeTX sensor names.  A fully populated,
    changing telemetry column is preferred over a duplicate placeholder that is blank
    or stuck at a constant value (for example the duplicated Ptch column seen in older
    ArduPilot/CRSF logs).
    """
    values: List[float] = []
    considered = 0
    for row in rows[:sample_limit]:
        considered += 1
        value = _parse_float(_clean_cell(row, idx))
        if value is not None and math.isfinite(float(value)):
            values.append(float(value))
    if not values:
        return {"coverage": 0.0, "unique": 0.0, "span": 0.0, "nonzero": 0.0, "score": -1.0}
    coverage = len(values) / max(1, considered)
    unique = len(set(round(v, 8) for v in values))
    span = max(values) - min(values)
    nonzero = sum(abs(v) > 1e-12 for v in values) / len(values)
    # Coverage matters most; dynamic/non-placeholder behaviour breaks duplicate-name ties.
    dynamic_bonus = min(1.0, math.log10(unique + 1.0) / 3.0)
    span_bonus = min(1.0, abs(span) / 10.0)
    score = coverage * 100.0 + dynamic_bonus * 8.0 + span_bonus * 4.0 + nonzero * 2.0
    return {"coverage": coverage, "unique": float(unique), "span": float(span), "nonzero": nonzero, "score": score}


def _best_numeric_col_index(header: List[str], rows: List[List[str]], targets: List[str]) -> Optional[int]:
    """Find the best numeric column for a semantic field, including duplicate headers.

    Target order still expresses naming preference (e.g. Ptch(rad) before generic Ptch),
    but duplicate columns with the same name are content-scored so a constant placeholder
    cannot mask the real sensor column.
    """
    for target in targets:
        exact = _matching_col_indices(header, [target], startswith=False)
        if exact:
            return max(exact, key=lambda i: (_numeric_column_profile(rows, i)["score"], i))
    for target in targets:
        partial = _matching_col_indices(header, [target], startswith=True)
        if partial:
            return max(partial, key=lambda i: (_numeric_column_profile(rows, i)["score"], i))
    return None


def _column_text_ratio(rows: List[List[str]], idx: int, predicate: Any, sample_limit: int = 2000) -> float:
    total = 0
    hits = 0
    for row in rows[:sample_limit]:
        text = _clean_cell(row, idx)
        if not text:
            continue
        total += 1
        try:
            if predicate(text):
                hits += 1
        except Exception:
            pass
    return hits / total if total else 0.0


def _looks_date_only(text: str) -> bool:
    return bool(re.fullmatch(r"\d{4}[-/]\d{1,2}[-/]\d{1,2}", str(text or "").strip()))


def _looks_time_only(text: str) -> bool:
    return bool(re.fullmatch(r"\d{1,2}:\d{2}:\d{2}(?:\.\d+)?", str(text or "").strip()))


def _parse_combined_datetime_text(text: str) -> Optional[datetime]:
    value = str(text or "").strip()
    if not value:
        return None
    candidate = value.replace("Z", "+00:00") if value.endswith("Z") else value
    try:
        return datetime.fromisoformat(candidate)
    except Exception:
        pass
    for fmt in (
        "%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d %H:%M:%S",
        "%Y/%m/%d %H:%M:%S.%f", "%Y/%m/%d %H:%M:%S",
        "%Y-%m-%dT%H:%M:%S.%f", "%Y-%m-%dT%H:%M:%S",
    ):
        try:
            return datetime.strptime(value, fmt)
        except Exception:
            pass
    return None


def _looks_combined_datetime(text: str) -> bool:
    value = str(text or "").strip()
    return (" " in value or "T" in value) and _parse_combined_datetime_text(value) is not None


def _select_datetime_columns(header: List[str], rows: List[List[str]]) -> Dict[str, Optional[int]]:
    """Distinguish EdgeTX local Date+Time from an additional GPS/CRSF UTC datetime field.

    Newer telemetry can create a second column also named ``Date`` whose cell contains
    both the GPS date and time.  The app keeps the ordinary date-only + time-only pair as
    the authoritative local log clock and records the combined GPS timestamp separately.
    """
    date_candidates = _matching_col_indices(header, ["Date"], startswith=False)
    time_candidates = _matching_col_indices(header, ["Time"], startswith=False)

    local_date_idx: Optional[int] = None
    if date_candidates:
        local_date_idx = max(
            date_candidates,
            key=lambda i: (_column_text_ratio(rows, i, _looks_date_only), -i),
        )
        if _column_text_ratio(rows, local_date_idx, _looks_date_only) <= 0.0:
            local_date_idx = date_candidates[0]

    local_time_idx: Optional[int] = None
    if time_candidates:
        local_time_idx = max(
            time_candidates,
            key=lambda i: (_column_text_ratio(rows, i, _looks_time_only), -i),
        )
        if _column_text_ratio(rows, local_time_idx, _looks_time_only) <= 0.0:
            local_time_idx = time_candidates[0]

    utc_datetime_idx: Optional[int] = None
    datetime_candidates: List[int] = []
    for i, _name in enumerate(header):
        if i == local_date_idx or i == local_time_idx:
            continue
        ratio = _column_text_ratio(rows, i, _looks_combined_datetime)
        if ratio >= 0.5:
            # Strong preference for a Date/GPS-time-labelled field; content is the final check.
            h = _normalise_header_name(header[i])
            label_bonus = 1.0 if ("date" in h or "time" in h or "gps" in h or "utc" in h) else 0.0
            datetime_candidates.append(i)
    if datetime_candidates:
        utc_datetime_idx = max(
            datetime_candidates,
            key=lambda i: (
                (1.0 if any(k in _normalise_header_name(header[i]) for k in ("date", "time", "gps", "utc")) else 0.0),
                _column_text_ratio(rows, i, _looks_combined_datetime),
                -i,
            ),
        )

    return {"date": local_date_idx, "time": local_time_idx, "utc_datetime": utc_datetime_idx}


def _filename_local_datetime(csv_path: str) -> Optional[datetime]:
    """Read the common EdgeTX filename timestamp as a local-time fallback anchor."""
    base = os.path.basename(csv_path)
    match = re.search(r"-(\d{4}-\d{2}-\d{2})-(\d{6})(?:\D|$)", base)
    if not match:
        return None
    try:
        return datetime.strptime(match.group(1) + match.group(2), "%Y-%m-%d%H%M%S")
    except Exception:
        return None


def _normalise_logged_heading_deg(value: Optional[float]) -> Optional[float]:
    """Recover CRSF/EdgeTX headings that wrapped through a signed 16-bit representation.

    Heading telemetry is conceptually 0..360 degrees.  Some logs expose values above
    about 327.67 degrees as negative numbers (hundredths of a degree interpreted as a
    signed integer).  Adding 655.36 degrees recovers the intended value before wrapping.
    """
    if value is None:
        return None
    x = float(value)
    if -327.68 <= x < 0.0:
        x += 655.36
    return x % 360.0


def _normalise_flight_mode_token(value: Any) -> str:
    mode = str(value or "").strip().upper()
    if not mode:
        return ""
    # Preserve the Betaflight failsafe token while ignoring suffix status markers such as '*'.
    if mode != "!FS!":
        mode = mode.rstrip("*?!")
    return mode


ARDUPILOT_STRONG_MODE_TOKENS = {
    "FBWA", "FBWB", "AUTO", "GUID", "GUIDED", "LOIT", "LOITER", "TKOF", "TAKEOFF",
    "CIRC", "CIRCLE", "CRUI", "CRUISE", "QSTB", "QHOV", "QLOIT", "QLAND", "QRTL",
    "THERMAL", "AUTOTUNE",
}
BETAFLIGHT_STRONG_MODE_TOKENS = {"AIR", "PASS", "CHIR"}
INAV_STRONG_MODE_TOKENS = {"WRTH", "LOTR", "HOLD", "CRUZ", "CRSH", "WP", "AH", "ANGH", "WAIT", "HRST", "GEO", "TURT"}

ARDUPILOT_PLANE_AUTO_THROTTLE_MODES = {
    "FBWB", "CRUI", "CRUISE", "AUTO", "GUID", "GUIDED", "CIRC", "CIRCLE", "LOIT", "LOITER",
    "RTL", "TKOF", "TAKEOFF", "LAND", "QLAND", "QRTL",
}
ARDUPILOT_COPTER_AUTONOMOUS_THROTTLE_MODES = {
    "AUTO", "GUID", "GUIDED", "LOIT", "LOITER", "RTL", "LAND", "ALTH", "ALTHOLD",
    "POSH", "POSHOLD", "CIRC", "CIRCLE", "FOLLOW", "SMARTRTL", "SMART", "BRAKE", "THROW",
}
BETAFLIGHT_AUTONOMOUS_THROTTLE_MODES = {"RTH", "POSH", "ALTH"}
INAV_AUTONOMOUS_THROTTLE_MODES = {"RTH", "WRTH", "LOTR", "HOLD", "CRUZ", "WP", "AH", "LAND", "GEO"}
# These mode labels imply controller-managed navigation/altitude/throttle regardless of
# which FC family supplied the CRSF text.  They provide a conservative fallback when a
# log contains too few firmware-specific mode strings for a confident stack ID.
UNIVERSAL_CONTROLLER_MANAGED_MODES = {
    "AUTO", "GUID", "GUIDED", "RTL", "RTH", "WRTH", "WP", "LAND",
    "LOIT", "LOITER", "LOTR", "HOLD", "CRUI", "CRUISE", "CRUZ",
    "POSH", "POSHOLD", "ALTH", "ALTHOLD", "AH", "CIRC", "CIRCLE",
    "TKOF", "TAKEOFF", "QLAND", "QRTL",
}
AUTONOMY_SIGNIFICANT_FRACTION = 0.10


def _detect_flight_stack_from_table(header: List[str], rows: List[List[str]]) -> Dict[str, Any]:
    """Infer ArduPilot / Betaflight / INAV primarily from official CRSF flight-mode strings."""
    fm_idx = _find_any_col_index(header, ["FM", "FlightMode", "Flight Mode", "Mode"])
    modes: List[str] = []
    if fm_idx is not None:
        for row in rows:
            token = _normalise_flight_mode_token(_clean_cell(row, fm_idx))
            if token:
                modes.append(token)
    unique_modes = sorted(set(modes))
    mode_set = set(unique_modes)
    ap_hits = sorted(mode_set & ARDUPILOT_STRONG_MODE_TOKENS)
    bf_hits = sorted(mode_set & BETAFLIGHT_STRONG_MODE_TOKENS)
    inav_hits = sorted(mode_set & INAV_STRONG_MODE_TOKENS)

    stack = "unknown"
    confidence = "low"
    reasons: List[str] = []
    scores = {"ardupilot": len(ap_hits), "betaflight": len(bf_hits), "inav": len(inav_hits)}
    if any(scores.values()):
        best = max(scores, key=lambda k: scores[k])
        ordered = sorted(scores.values(), reverse=True)
        if scores[best] > 0 and (len(ordered) < 2 or ordered[0] > ordered[1]):
            stack = best
            confidence = "high" if scores[best] >= 2 or best in ("ardupilot", "betaflight") else "medium"
    if stack == "ardupilot": reasons.append("ArduPilot CRSF flight-mode token(s): " + ", ".join(ap_hits))
    elif stack == "betaflight": reasons.append("Betaflight CRSF flight-mode token(s): " + ", ".join(bf_hits))
    elif stack == "inav": reasons.append("INAV CRSF flight-mode token(s): " + ", ".join(inav_hits))

    # Conservative fallbacks for ambiguous mode strings.  Do not guess from column order.
    names = {_normalise_header_name(x) for x in header}
    has_vspd = any(name.startswith("vspd") for name in names)
    has_temp = any(name.startswith("temp") for name in names)
    datetime_info = _select_datetime_columns(header, rows)
    if stack == "unknown" and ("FBWA" in mode_set or "FBWB" in mode_set or "TKOF" in mode_set):
        stack, confidence = "ardupilot", "high"
        reasons.append("ArduPilot fixed-wing mode naming detected")
    elif stack == "unknown" and "AIR" in mode_set:
        stack, confidence = "betaflight", "high"
        reasons.append("Betaflight AIR mode naming detected")
    elif stack == "unknown" and inav_hits:
        stack, confidence = "inav", "high"
        reasons.append("INAV navigation mode naming detected")
    elif stack == "unknown" and datetime_info.get("utc_datetime") is not None and (has_vspd or has_temp):
        reasons.append("new CRSF GPS-time/vario/temperature telemetry present, but it is not firmware-specific by itself")

    return {
        "stack": stack,
        "confidence": confidence,
        "reason": "; ".join(reasons) if reasons else "no firmware-specific flight-mode token was found",
        "modes": unique_modes,
        "mode_index": fm_idx,
    }


def detect_flight_stack_csv(csv_path: Optional[str]) -> Dict[str, Any]:
    if not csv_path:
        return {"stack": "unknown", "confidence": "low", "reason": "no CSV path", "modes": [], "mode_index": None}
    try:
        with open(csv_path, "r", encoding="utf-8-sig", errors="replace", newline="") as f:
            dialect = sniff_dialect(f)
            reader = csv.reader(f, dialect)
            header = next(reader, None)
            if not header:
                raise ValueError("empty CSV")
            rows = [list(row) for row in reader]
        return _detect_flight_stack_from_table(header, rows)
    except Exception as exc:
        return {"stack": "unknown", "confidence": "low", "reason": f"flight-stack inspection failed: {exc}", "modes": [], "mode_index": None}


def _pearson_pairs(pairs: List[Tuple[float, float]]) -> Optional[float]:
    if len(pairs) < 10:
        return None
    xs = [float(a) for a, _b in pairs]
    ys = [float(b) for _a, b in pairs]
    mx = sum(xs) / len(xs); my = sum(ys) / len(ys)
    sx = sum((x - mx) ** 2 for x in xs); sy = sum((y - my) ** 2 for y in ys)
    if sx <= 1e-12 or sy <= 1e-12:
        return None
    cov = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    return cov / math.sqrt(sx * sy)


def _autonomous_mode_set_for_stack(stack: str, observed_modes: List[str]) -> Tuple[set, str]:
    mode_set = set(observed_modes)
    if stack == "ardupilot":
        looks_plane = bool(mode_set & {"FBWA", "FBWB", "TKOF", "TAKEOFF", "CRUI", "CRUISE", "CIRC"})
        return (ARDUPILOT_PLANE_AUTO_THROTTLE_MODES if looks_plane else ARDUPILOT_COPTER_AUTONOMOUS_THROTTLE_MODES), ("plane" if looks_plane else "copter")
    if stack == "betaflight":
        return BETAFLIGHT_AUTONOMOUS_THROTTLE_MODES, "betaflight"
    if stack == "inav":
        return INAV_AUTONOMOUS_THROTTLE_MODES, "inav"
    return UNIVERSAL_CONTROLLER_MANAGED_MODES, "unknown/ambiguous firmware"


def _assess_autonomy_from_records(records: List[Dict[str, Any]], flight_stack: Dict[str, Any], sample_period: Optional[float] = None) -> Dict[str, Any]:
    """Classify whether RC throttle is likely not the actual motor-throttle history.

    Flight-mode evidence is primary.  Throttle/current correlation is retained only as a
    secondary diagnostic because propeller load, airspeed, battery voltage and controller
    behaviour can weaken that correlation even during manual flight.
    """
    stack = str(flight_stack.get("stack", "unknown"))
    observed_modes = sorted({str(r.get("flight_mode") or "") for r in records if r.get("flight_mode")})
    auto_modes, vehicle_kind = _autonomous_mode_set_for_stack(stack, observed_modes)
    default_dt = float(sample_period or 0.0) if sample_period else 0.0
    if default_dt <= 0:
        deltas = []
        for i in range(1, len(records)):
            dt = float(records[i].get("elapsed_s", 0.0)) - float(records[i - 1].get("elapsed_s", 0.0))
            if 0 < dt < 10:
                deltas.append(dt)
        default_dt = _median(deltas) or 0.2

    total_s = 0.0
    mode_known_s = 0.0
    auto_s = 0.0
    autonomous_modes_seen: set = set()
    for i, record in enumerate(records):
        if i + 1 < len(records):
            dt = float(records[i + 1].get("elapsed_s", 0.0)) - float(record.get("elapsed_s", 0.0))
            if dt <= 0 or dt > max(10.0, default_dt * 20.0):
                dt = default_dt
        else:
            dt = default_dt
        total_s += max(0.0, dt)
        mode = str(record.get("flight_mode") or "")
        if mode:
            mode_known_s += max(0.0, dt)
            if mode in auto_modes:
                auto_s += max(0.0, dt)
                autonomous_modes_seen.add(mode)

    fraction = auto_s / total_s if total_s > 0 else 0.0
    mode_coverage = mode_known_s / total_s if total_s > 0 else 0.0
    pairs = [
        (float(r["throttle_pct"]), float(r["curr"]))
        for r in records
        if r.get("throttle_pct") is not None and r.get("curr") is not None
    ]
    corr = _pearson_pairs(pairs)

    if mode_coverage >= 0.25 and fraction >= AUTONOMY_SIGNIFICANT_FRACTION:
        classification = "semi_autonomous"
        reason = f"{fraction * 100.0:.1f}% of logged time was in mode(s) where throttle/motor output is controller-managed"
    elif mode_coverage >= 0.25:
        classification = "pilot_throttle_representative"
        reason = f"only {fraction * 100.0:.1f}% of logged time was in controller-managed throttle/navigation modes"
    else:
        classification = "unknown"
        reason = "flight-mode coverage was insufficient for a confident autonomy classification"

    if corr is not None:
        reason += f"; RC-throttle/current correlation={corr:.2f} (secondary diagnostic only)"
    return {
        "classification": classification,
        "autonomous_fraction": fraction,
        "autonomous_seconds": auto_s,
        "mode_coverage": mode_coverage,
        "autonomous_modes": sorted(autonomous_modes_seen),
        "throttle_current_corr": corr,
        "vehicle_kind": vehicle_kind,
        "reason": reason,
    }


def assess_flight_autonomy_csv(csv_path: Optional[str], throttle_col_name: str = "CH3(us)") -> Dict[str, Any]:
    """Lightweight public helper used by map/stat menus before a full export starts."""
    if not csv_path:
        return {"classification": "unknown", "reason": "no CSV path", "autonomous_fraction": 0.0, "autonomous_modes": []}
    try:
        data = _read_telemetry_records(csv_path, throttle_col_name=throttle_col_name)
        return dict(data.get("autonomy") or {})
    except Exception as exc:
        return {"classification": "unknown", "reason": f"autonomy inspection failed: {exc}", "autonomous_fraction": 0.0, "autonomous_modes": []}


def _parse_gps_cell(gps_cell: str) -> Optional[Tuple[float, float]]:
    """Parse GPS cell in '<lat> <lon>' format."""
    gps_cell = (gps_cell or "").strip()
    if not gps_cell:
        return None

    parts = gps_cell.split()
    if len(parts) < 2:
        return None

    try:
        lat = float(parts[0])
        lon = float(parts[1])
    except Exception:
        return None

    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return None
    if lat == 0.0 and lon == 0.0:
        return None

    return lat, lon


def _parse_datetime_value(date_text: str, time_text: str) -> Optional[float]:
    """Parse the *local* EdgeTX Date + Time into timezone-neutral seconds.

    Only differences are needed for flight timing.  Using a naive ordinal-based value avoids
    letting the computer's current OS timezone/DST rules change elapsed-time calculations.
    A separate GPS/CRSF UTC datetime, when present, is intentionally handled elsewhere.
    """
    date_text = (date_text or "").strip()
    time_text = (time_text or "").strip()
    if not time_text:
        return None

    def seconds_for(dt: datetime) -> float:
        return float(dt.toordinal() * 86400 + dt.hour * 3600 + dt.minute * 60 + dt.second) + dt.microsecond / 1_000_000.0

    if date_text:
        combo = f"{date_text} {time_text}"
        try:
            return seconds_for(datetime.fromisoformat(combo))
        except Exception:
            pass
        for fmt in ("%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d %H:%M:%S", "%Y/%m/%d %H:%M:%S.%f", "%Y/%m/%d %H:%M:%S"):
            try:
                return seconds_for(datetime.strptime(combo, fmt))
            except Exception:
                pass

    for fmt in ("%H:%M:%S.%f", "%H:%M:%S"):
        try:
            dt = datetime.strptime(time_text, fmt)
            return float(dt.hour * 3600 + dt.minute * 60 + dt.second) + (dt.microsecond / 1_000_000.0)
        except Exception:
            pass
    return None

def _format_time_for_display(date_text: str, time_text: str) -> str:
    """Display the time from the CSV, keeping milliseconds if present."""
    time_text = (time_text or "").strip()
    date_text = (date_text or "").strip()
    if time_text:
        return time_text
    if date_text:
        return date_text
    return "n/a"


def haversine_m(a: List[float] | Tuple[float, float], b: List[float] | Tuple[float, float]) -> float:
    """Distance in metres between [lat, lon] points."""
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    x = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 6371000.0 * 2 * math.atan2(math.sqrt(x), math.sqrt(1 - x))


def _format_distance(meters: Optional[float]) -> str:
    if meters is None:
        return "n/a"
    if meters < 1000:
        return f"{meters:.0f} m"
    return f"{meters / 1000:.2f} km"


def _format_duration(seconds: Optional[float]) -> str:
    if seconds is None:
        return "n/a"
    # Round once to tenths first so 59.96 seconds displays as 1m 00.0s,
    # not as a confusing 0m 60.0s.
    total_tenths = int(round(max(0.0, float(seconds)) * 10.0))
    hours = total_tenths // 36000
    rem = total_tenths % 36000
    minutes = rem // 600
    secs_tenths = rem % 600
    secs = secs_tenths / 10.0
    if hours:
        return f"{hours}h {minutes:02d}m {secs:04.1f}s"
    if minutes:
        return f"{minutes}m {secs:04.1f}s"
    return f"{secs:.1f}s"


def _format_num(value: Optional[float], decimals: int = 1, unit: str = "") -> str:
    if value is None:
        return "n/a"
    if decimals == 0:
        s = f"{value:.0f}"
    else:
        s = f"{value:.{decimals}f}"
    return f"{s}{unit}"


def _list_stats(values: List[float]) -> Dict[str, Optional[float]]:
    if not values:
        return {"count": 0, "avg": None, "min": None, "max": None, "last": None}
    return {
        "count": len(values),
        "avg": sum(values) / len(values),
        "min": min(values),
        "max": max(values),
        "last": values[-1],
    }

def _valid_rssi_value(value: Optional[float]) -> Optional[float]:
    """
    Clean an RSSI dBm value.

    EdgeTX/CRSF logs often use 0 in 2RSS when there is no true second receiver chain.
    RSSI dBm should normally be negative, so 0 is treated as "not available" here.
    """
    if value is None:
        return None
    v = float(value)
    if v == 0.0:
        return None
    return v


def _best_rssi_from_values(rssi1: Optional[float], rssi2: Optional[float]) -> Optional[float]:
    """
    Return the best usable RSSI dBm value for one timestamp.

    - Single-chain logs: if 2RSS is 0/missing, use 1RSS.
    - True diversity logs: if both chains have real non-zero values, use the higher/better value.
    - If one chain is 0 and the other is real, use the real one.
    """
    values: List[float] = []
    v1 = _valid_rssi_value(rssi1)
    v2 = _valid_rssi_value(rssi2)
    if v1 is not None:
        values.append(v1)
    if v2 is not None:
        values.append(v2)
    return max(values) if values else None


def _detect_rssi_diversity_from_columns(rssi1_values: List[float], rssi2_values: List[float]) -> bool:
    """
    Detect true receiver diversity from RSSI columns.

    A log is treated as diversity only when both 1RSS and 2RSS contain real non-zero
    samples. If 2RSS is present but all zeros, it is treated as a single-chain log.
    """
    has_1rss = any(_valid_rssi_value(v) is not None for v in rssi1_values)
    has_2rss = any(_valid_rssi_value(v) is not None for v in rssi2_values)
    return bool(has_1rss and has_2rss)



def _unwrap_time_values(times: List[float], date_used: bool) -> List[float]:
    """
    If times are time-of-day only and the flight crosses midnight, make them monotonic.
    If date was used, timestamps are already absolute and should not need unwrapping.
    """
    if date_used or not times:
        return times

    adjusted: List[float] = []
    offset = 0.0
    previous: Optional[float] = None
    for t in times:
        candidate = t + offset
        # If time jumps backwards by more than 12 hours, assume midnight rollover.
        if previous is not None and candidate < previous - 12 * 3600:
            offset += 24 * 3600
            candidate = t + offset
        adjusted.append(candidate)
        previous = candidate
    return adjusted

def _format_date_plain(date_text: str) -> str:
    """Format a CSV Date value like '2026-12-22' as 'DEC. 22, 2026'."""
    date_text = (date_text or "").strip()
    if not date_text:
        return "n/a"

    parsed = None
    for fmt in ("%Y-%m-%d", "%Y/%m/%d", "%m/%d/%Y", "%d/%m/%Y"):
        try:
            parsed = datetime.strptime(date_text, fmt)
            break
        except Exception:
            pass

    if parsed is None:
        try:
            parsed = datetime.fromisoformat(date_text)
        except Exception:
            return date_text

    month = parsed.strftime("%b").upper() + "."
    return f"{month} {parsed.day}, {parsed.year}"


def _filter_relative_altitude_spikes(samples: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """
    Remove Betaflight/EdgeTX altitude samples that are clearly MSL/elevation glitches
    inside an otherwise relative-altitude log.

    This handles all of these common cases:
      - first samples are takeoff elevation, then Alt(m) resets near 0
      - first row is 0, then a few takeoff-elevation samples, then reset near 0
      - a later brief jump back to takeoff elevation/MSL, including at the very end
      - normal real climbs are preserved because they do not jump hundreds of metres
        in one sample and then immediately jump back.
    """
    if len(samples) < 2:
        return list(samples)

    def _alt_at(index: int) -> Optional[float]:
        try:
            return float(samples[index].get("alt", 0.0))
        except Exception:
            return None

    filtered: List[Dict[str, Any]] = []
    i = 0
    n = len(samples)

    while i < n:
        alt = _alt_at(i)
        if alt is None:
            i += 1
            continue

        prev_alt: Optional[float] = None
        if filtered:
            try:
                prev_alt = float(filtered[-1].get("alt", 0.0))
            except Exception:
                prev_alt = None

        # Detect a short run of nearly identical MSL-looking values.
        # EdgeTX often logs the takeoff elevation as repeated identical samples.
        if abs(alt) >= ALT_MSL_LOOKING_THRESHOLD_M:
            j = i + 1
            while j < n:
                next_run_alt = _alt_at(j)
                if next_run_alt is None:
                    break
                # Keep the run together only while it looks like the same MSL/elevation glitch.
                if abs(next_run_alt - alt) <= 50.0:
                    j += 1
                    continue
                break

            run_len = j - i
            next_alt = _alt_at(j) if j < n else None

            jumped_up = prev_alt is not None and abs(alt - prev_alt) > ALT_RELATIVE_SPIKE_JUMP_THRESHOLD_M
            starts_with_msl_then_reset = (
                prev_alt is None
                and run_len <= 120
                and next_alt is not None
                and abs(next_alt) <= ALT_RELATIVE_ZERO_THRESHOLD_M
                and abs(alt - next_alt) > ALT_RELATIVE_SPIKE_JUMP_THRESHOLD_M
            )
            short_jump_then_return = (
                jumped_up
                and run_len <= 120
                and next_alt is not None
                and abs(alt - next_alt) > ALT_RELATIVE_SPIKE_JUMP_THRESHOLD_M
                and (
                    abs(next_alt) <= ALT_RELATIVE_ZERO_THRESHOLD_M
                    or (prev_alt is not None and abs(next_alt - prev_alt) <= ALT_RELATIVE_SPIKE_JUMP_THRESHOLD_M)
                )
            )
            short_jump_at_end = (
                jumped_up
                and run_len <= 120
                and next_alt is None
            )

            if starts_with_msl_then_reset or short_jump_then_return or short_jump_at_end:
                i = j
                continue

        filtered.append(samples[i])
        i += 1

    return filtered


def _adjust_altitude_samples_for_stats(samples: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Prepare altitude samples for stats.

    Betaflight/EdgeTX:
      Alt(m) is normally relative altitude, but some logs briefly contain takeoff
      elevation/MSL values before the relative-altitude stream begins, or briefly jump
      back to MSL later. Those MSL-looking blocks are removed before calculating stats.

    ArduPilot-like:
      Alt(m) stays above-sea-level for the entire log. Those samples are converted to
      relative altitude by subtracting the first altitude sample, and the first sample is
      reported as takeoff elevation.
    """
    if not samples:
        return {"samples": [], "source": "unknown", "takeoff_elevation_m": None}

    if len(samples) < 2:
        return {"samples": samples, "source": "relative", "takeoff_elevation_m": None}

    raw_values = [float(s.get("alt", 0.0)) for s in samples]
    first = raw_values[0]
    has_near_zero = any(abs(v) <= ALT_RELATIVE_ZERO_THRESHOLD_M for v in raw_values)
    high_fraction = sum(1 for v in raw_values if abs(v) >= ALT_MSL_LOOKING_THRESHOLD_M) / max(1, len(raw_values))

    # ArduPilot-like ASL: the whole log stays at elevation/MSL and never resets near 0.
    # A high fraction check avoids misclassifying Betaflight logs with a few MSL glitch samples.
    if abs(first) >= ALT_MSL_LOOKING_THRESHOLD_M and not has_near_zero and high_fraction >= 0.80:
        adjusted: List[Dict[str, Any]] = []
        for sample in samples:
            new_sample = dict(sample)
            new_sample["alt"] = float(sample.get("alt", 0.0)) - first
            adjusted.append(new_sample)
        return {"samples": adjusted, "source": "ardupilot_asl", "takeoff_elevation_m": first}

    filtered = _filter_relative_altitude_spikes(samples)
    source = "relative"
    if len(filtered) != len(samples):
        source = "relative_msl_glitches_removed"
    return {"samples": filtered, "source": source, "takeoff_elevation_m": None}


def _format_alt_time_to_max(seconds: Optional[float], display_time: str) -> str:
    if seconds is None and not display_time:
        return "n/a"
    if seconds is None:
        return escape(str(display_time))
    if display_time:
        return f"{_format_duration(seconds)} ({escape(str(display_time))})"
    return _format_duration(seconds)


def read_flight_data(
    csv_path: str,
    gps_col_name: str = "GPS",
    sats_col_name: str = "sats",
    min_sats: int = MIN_SATS,
    dedup_decimals: int = DEDUP_DECIMALS,
    throttle_col_name: str = "CH3(us)",
) -> Dict[str, Any]:
    """
    Reads the CSV once and returns:
      - track segments for the map
      - parsing/continuity stats
      - calculated flight stats for optional display
    """
    segments: List[List[List[float]]] = []
    current: List[List[float]] = []

    rows = 0
    kept = 0
    bad_gps = 0
    missing_gps = 0
    low_sats = 0
    four_sat_rows_total = 0
    four_sat_rows_kept = 0
    deduped = 0
    decimated = 0

    last_key: Optional[Tuple[float, float]] = None

    # Telemetry lists for stats
    time_values: List[float] = []
    time_display_values: List[str] = []
    date_used_for_time = False
    first_date_text = ""

    alt_samples: List[Dict[str, Any]] = []

    numeric_lists: Dict[str, List[float]] = {
        "gspd": [],
        "sats": [],
        "rqly": [],
        "rsnr": [],
        "rssi_best": [],
        "rssi1_raw": [],
        "rssi2_raw": [],
        "tpwr": [],
        "rxbt": [],
        "curr": [],
        "power_w": [],
        "capa": [],
        "efficiency_mAh_km": [],
        "alt": [],
        "vspd_logged": [],
        "temperature": [],
        "throttle_us": [],
        "throttle_pct": [],
    }

    # For cumulative efficiency calculations.
    eff_prev_point: Optional[List[float]] = None
    eff_cum_distance_m = 0.0
    eff_start_capa: Optional[float] = None

    def close_segment():
        nonlocal current, last_key
        if current:
            segments.append(current)
        current = []
        last_key = None

    with open(csv_path, "r", encoding="utf-8-sig", errors="replace", newline="") as f:
        dialect = sniff_dialect(f)
        reader = csv.reader(f, dialect)

        try:
            header = next(reader)
        except StopIteration:
            return _empty_flight_data()
        data_rows = [list(row) for row in reader]

        gps_idx = _find_any_col_index(header, [gps_col_name, "GPS", "GPS Coordinates"])
        sats_idx = _best_numeric_col_index(header, data_rows, [sats_col_name, "Sats", "Satellites"])

        datetime_idx = _select_datetime_columns(header, data_rows)
        time_idx = datetime_idx.get("time")
        date_idx = datetime_idx.get("date")
        filename_dt = _filename_local_datetime(csv_path)
        filename_date_text = filename_dt.strftime("%Y-%m-%d") if filename_dt is not None else ""

        # Flexible semantic telemetry lookup; column order is irrelevant and duplicate names
        # are content-scored when necessary.
        col = {
            "gspd": _best_numeric_col_index(header, data_rows, ["GSpd(kmh)", "GSpd(km/h)", "GSpd", "GroundSpeed"]),
            "rqly": _best_numeric_col_index(header, data_rows, ["RQly(%)", "RQly", "LQ"]),
            "rsnr": _best_numeric_col_index(header, data_rows, ["RSNR(dB)", "RSNR(dBm)", "RSNR", "SNR"]),
            "tpwr": _best_numeric_col_index(header, data_rows, ["TPWR(mW)", "TPWR", "TPWR(W)"]),
            "rxbt": _best_numeric_col_index(header, data_rows, ["RxBt(V)", "RxBt", "Voltage(V)"]),
            "curr": _best_numeric_col_index(header, data_rows, ["Curr(A)", "Curr", "Current(A)"]),
            "capa": _best_numeric_col_index(header, data_rows, ["Capa(mAh)", "Capa", "Capacity(mAh)"]),
            "alt": _best_numeric_col_index(header, data_rows, ["Alt(m)", "Alt", "alt (m)", "Altitude(m)"]),
            "vspd_logged": _best_numeric_col_index(header, data_rows, ["VSpd(m/s)", "VSpd", "VSpeed(m/s)", "VerticalSpeed(m/s)", "Vario(m/s)"]),
            "temperature": _best_numeric_col_index(header, data_rows, ["Temp(°C)", "Temp(C)", "Temp(°)", "Temp", "Temperature(°C)", "Temperature"]),
        }

        rssi1_idx = _best_numeric_col_index(header, data_rows, ["1RSS(dB)", "1RSS(dBm)", "1RSS", "RSSI1"])
        rssi2_idx = _best_numeric_col_index(header, data_rows, ["2RSS(dB)", "2RSS(dBm)", "2RSS", "RSSI2"])
        throttle_col_name = throttle_col_name or "CH3(us)"
        throttle_idx = _best_numeric_col_index(header, data_rows, [throttle_col_name, throttle_col_name.replace("(us)", ""), "CH3(us)", "CH3"])

        if gps_idx is None:
            return _empty_flight_data(rows=rows)

        for row in data_rows:
            rows += 1

            # --- Time collection ---
            date_text = _clean_cell(row, date_idx)
            time_text = _clean_cell(row, time_idx)
            effective_date = date_text or (filename_date_text if time_text else "")
            if effective_date and not first_date_text:
                first_date_text = effective_date
            parsed_time = _parse_datetime_value(effective_date, time_text)
            display_time = _format_time_for_display(effective_date, time_text)
            if parsed_time is not None:
                time_values.append(parsed_time)
                time_display_values.append(display_time)
                if effective_date:
                    date_used_for_time = True

            # --- Numeric telemetry collection ---
            row_values: Dict[str, Optional[float]] = {}
            for key, idx in col.items():
                value = _parse_float(_clean_cell(row, idx))
                row_values[key] = value
                if value is not None:
                    numeric_lists[key].append(value)

            # Power in watts = current amps * receiver battery voltage.
            curr_value = row_values.get("curr")
            rxbt_value = row_values.get("rxbt")
            if curr_value is not None and rxbt_value is not None:
                numeric_lists["power_w"].append(curr_value * rxbt_value)

            # Altitude samples with time so time-to-max-altitude can be calculated.
            alt_value = row_values.get("alt")
            if alt_value is not None:
                alt_samples.append({"alt": alt_value, "time": parsed_time, "display": display_time})

            # RSSI dBm handling:
            # - 0 in 2RSS usually means "no second receiver chain", not a real 0 dBm signal.
            # - If both chains have real non-zero values, use the better/higher value per timestamp.
            rssi1 = _parse_float(_clean_cell(row, rssi1_idx))
            rssi2 = _parse_float(_clean_cell(row, rssi2_idx))
            if rssi1 is not None:
                numeric_lists["rssi1_raw"].append(rssi1)
            if rssi2 is not None:
                numeric_lists["rssi2_raw"].append(rssi2)
            best_rssi = _best_rssi_from_values(rssi1, rssi2)
            if best_rssi is not None:
                numeric_lists["rssi_best"].append(best_rssi)

            throttle_value = _parse_float(_clean_cell(row, throttle_idx))
            if throttle_value is not None:
                numeric_lists["throttle_us"].append(throttle_value)
                throttle = (throttle_value - THROTTLE_MIN_US) / (THROTTLE_MAX_US - THROTTLE_MIN_US) * 100.0
                throttle = max(0.0, min(100.0, throttle))
                numeric_lists["throttle_pct"].append(throttle)

            # Sats list from the actual sats column, if present.
            sats_val = None
            if sats_idx is not None:
                sats_val = _parse_sats(_clean_cell(row, sats_idx))
                if sats_val is not None:
                    numeric_lists["sats"].append(sats_val)
                    if int(round(sats_val)) == 4:
                        four_sat_rows_total += 1

            # --- Sats check for track continuity ---
            sats_ok = True
            if sats_idx is not None:
                if sats_idx >= len(row):
                    sats_ok = False
                else:
                    if sats_val is None:
                        sats_ok = False
                    elif sats_val < float(min_sats):
                        sats_ok = False

            # --- GPS check for track continuity ---
            if gps_idx >= len(row):
                close_segment()
                missing_gps += 1
                continue

            gps_cell = _clean_cell(row, gps_idx)
            if not gps_cell:
                close_segment()
                missing_gps += 1
                continue

            gps_parsed = _parse_gps_cell(gps_cell)
            if gps_parsed is None:
                close_segment()
                bad_gps += 1
                continue

            lat, lon = gps_parsed

            # --- Apply continuity rules ---
            if not sats_ok:
                close_segment()
                low_sats += 1
                continue

            current_point = [lat, lon]
            if sats_val is not None and int(round(sats_val)) == 4:
                four_sat_rows_kept += 1

            # Efficiency samples based on cumulative capacity used divided by cumulative distance travelled.
            # Use (current Capa - first Capa) so non-zero starting Capa does not skew the result.
            if eff_prev_point is not None:
                eff_cum_distance_m += haversine_m(eff_prev_point, current_point)
            eff_prev_point = current_point
            capa_value = row_values.get("capa")
            if capa_value is not None:
                if eff_start_capa is None:
                    eff_start_capa = capa_value
                capacity_used_for_eff = max(0.0, capa_value - eff_start_capa)
                if eff_cum_distance_m >= 100.0:
                    numeric_lists["efficiency_mAh_km"].append(capacity_used_for_eff / (eff_cum_distance_m / 1000.0))

            # --- Dedupe + append ---
            key = (round(lat, dedup_decimals), round(lon, dedup_decimals))
            if last_key == key:
                deduped += 1
                continue

            last_key = key
            current.append([key[0], key[1]])
            kept += 1

    close_segment()

    # Safety: if insanely large, decimate evenly (after dedupe)
    total_points = sum(len(seg) for seg in segments)
    if total_points > HARD_POINT_LIMIT and total_points > 0:
        step = max(1, total_points // HARD_POINT_LIMIT)
        new_segments: List[List[List[float]]] = []
        for seg in segments:
            if not seg:
                continue
            dec = seg[::step]
            if dec:
                new_segments.append(dec)
        segments = new_segments
        decimated = 1

    parse_stats = {
        "rows": rows,
        "kept": kept,
        "bad_gps": bad_gps,
        "missing_gps": missing_gps,
        "low_sats": low_sats,
        "four_sat_rows_total": four_sat_rows_total,
        "four_sat_rows_kept": four_sat_rows_kept,
        "min_sats": int(min_sats),
        "deduped": deduped,
        "segments": len(segments),
        "decimated": decimated,
    }

    flight_stats = compute_flight_stats(
        segments=segments,
        parse_stats=parse_stats,
        numeric_lists=numeric_lists,
        time_values=time_values,
        time_display_values=time_display_values,
        date_used_for_time=date_used_for_time,
        flight_date_text=first_date_text,
        alt_samples=alt_samples,
    )
    flight_stats["flight_stack"] = detect_flight_stack_csv(csv_path)
    flight_stats["autonomy"] = assess_flight_autonomy_csv(csv_path, throttle_col_name)

    return {
        "segments": segments,
        "parse_stats": parse_stats,
        "flight_stats": flight_stats,
    }


def _empty_flight_data(rows: int = 0) -> Dict[str, Any]:
    parse_stats = {
        "rows": rows,
        "kept": 0,
        "bad_gps": 0,
        "missing_gps": 0,
        "low_sats": 0,
        "deduped": 0,
        "segments": 0,
        "decimated": 0,
    }
    return {
        "segments": [],
        "parse_stats": parse_stats,
        "flight_stats": {
            "parse": parse_stats,
            "distance": {},
            "time": {},
            "numeric": {},
        },
    }


def compute_flight_stats(
    segments: List[List[List[float]]],
    parse_stats: Dict[str, int],
    numeric_lists: Dict[str, List[float]],
    time_values: List[float],
    time_display_values: List[str],
    date_used_for_time: bool,
    flight_date_text: str = "",
    alt_samples: Optional[List[Dict[str, Any]]] = None,
) -> Dict[str, Any]:
    """Calculate flight stats from map segments and telemetry values."""
    all_points: List[List[float]] = [pt for seg in segments for pt in seg]

    total_distance_m = 0.0
    for seg in segments:
        for a, b in zip(seg, seg[1:]):
            total_distance_m += haversine_m(a, b)

    max_distance_home_m: Optional[float] = None
    if all_points:
        home = all_points[0]
        max_distance_home_m = max(haversine_m(home, pt) for pt in all_points)

    adjusted_times = _unwrap_time_values(time_values, date_used_for_time)
    duration_s: Optional[float] = None
    logging_rate_hz: Optional[float] = None
    takeoff_time_value: Optional[float] = adjusted_times[0] if adjusted_times else None
    if len(adjusted_times) >= 2:
        duration_s = max(0.0, adjusted_times[-1] - adjusted_times[0])
        if duration_s > 0:
            logging_rate_hz = (len(adjusted_times) - 1) / duration_s

    numeric_lists_for_stats = {key: list(values) for key, values in numeric_lists.items()}

    alt_samples = alt_samples or []
    altitude_adjustment = _adjust_altitude_samples_for_stats(alt_samples)
    clean_alt_samples = altitude_adjustment["samples"]
    if clean_alt_samples:
        numeric_lists_for_stats["alt"] = [float(sample["alt"]) for sample in clean_alt_samples]
    elif "alt" in numeric_lists_for_stats:
        numeric_lists_for_stats["alt"] = []

    numeric_stats = {key: _list_stats(values) for key, values in numeric_lists_for_stats.items()}

    # Avg efficiency over the whole flight using consumed capacity and total distance.
    capa_stats = numeric_stats.get("capa", {})
    avg_efficiency = None
    capacity_used_total = None
    if capa_stats.get("max") is not None and capa_stats.get("min") is not None:
        capacity_used_total = max(0.0, float(capa_stats["max"]) - float(capa_stats["min"]))
    if total_distance_m > 0 and capacity_used_total is not None:
        avg_efficiency = capacity_used_total / (total_distance_m / 1000.0)

    max_alt_info = {
        "alt": None,
        "time_to_max_s": None,
        "display": "",
        "source": altitude_adjustment.get("source", "unknown"),
        "takeoff_elevation_m": altitude_adjustment.get("takeoff_elevation_m"),
    }
    if clean_alt_samples:
        max_sample = max(clean_alt_samples, key=lambda sample: float(sample.get("alt", 0.0)))
        max_alt_info["alt"] = float(max_sample.get("alt", 0.0))
        max_alt_info["display"] = str(max_sample.get("display", ""))
        max_time = max_sample.get("time")
        if isinstance(max_time, (int, float)) and takeoff_time_value is not None:
            max_alt_info["time_to_max_s"] = max(0.0, float(max_time) - float(takeoff_time_value))

    return {
        "parse": parse_stats,
        "time": {
            "date": _format_date_plain(flight_date_text),
            "takeoff": time_display_values[0] if time_display_values else "n/a",
            "landing": time_display_values[-1] if time_display_values else "n/a",
            "duration_s": duration_s,
            "logging_rate_hz": logging_rate_hz,
            "time_samples": len(time_values),
        },
        "distance": {
            "total_m": total_distance_m if all_points else None,
            "max_home_m": max_distance_home_m,
            "point_count": len(all_points),
        },
        "efficiency": {
            "avg_mAh_per_km": avg_efficiency,
            "max_mAh_per_km": numeric_stats.get("efficiency_mAh_km", {}).get("max"),
            "capacity_used_mAh": capacity_used_total,
        },
        "rssi": {
            "diversity": _detect_rssi_diversity_from_columns(
                numeric_lists_for_stats.get("rssi1_raw", []),
                numeric_lists_for_stats.get("rssi2_raw", []),
            ),
        },
        "altitude": max_alt_info,
        "numeric": numeric_stats,
    }




def _four_sat_warning_text(parse_info: Dict[str, Any]) -> str:
    """Return the map/report warning for present four-satellite samples."""
    four_total = int(parse_info.get("four_sat_rows_total", 0) or 0)
    if four_total <= 0:
        return ""
    min_sats = int(parse_info.get("min_sats", MIN_SATS) or MIN_SATS)
    four_kept = int(parse_info.get("four_sat_rows_kept", 0) or 0)
    if min_sats <= RELAXED_MIN_SATS and four_kept > 0:
        return "Warning: 4-satellite GPS data included; position and especially GPS altitude may be less reliable."
    return "Warning: 4-satellite GPS data excluded; position and especially GPS altitude may be less reliable."

def _flight_stat_lines(
    flight_stats: Dict[str, Any],
    groups: List[str],
    privacy_config: Dict[str, Any],
    throttle_col_name: str = "CH3(us)",
) -> List[str]:
    lines: List[str] = []
    selected = set(groups)

    time_s = flight_stats.get("time", {})
    dist = flight_stats.get("distance", {})
    numeric = flight_stats.get("numeric", {})
    efficiency = flight_stats.get("efficiency", {})
    altitude_info = flight_stats.get("altitude", {})

    if "default" in selected:
        gspd = numeric.get("gspd", {})
        rsnr = numeric.get("rsnr", {})
        lines.extend([
            "<b>Flight stats</b>",
            f"Date: {escape(str(time_s.get('date', 'n/a')))}",
            f"Takeoff: {escape(str(time_s.get('takeoff', 'n/a')))}",
            f"Landing: {escape(str(time_s.get('landing', 'n/a')))}",
            f"Time in air: {_format_duration(time_s.get('duration_s'))}",
            f"Avg speed: {_format_num(gspd.get('avg'), 1, ' km/h')}",
            f"Max speed: {_format_num(gspd.get('max'), 1, ' km/h')}",
            f"Max distance from home: {_format_distance(dist.get('max_home_m'))}",
            f"Total distance: {_format_distance(dist.get('total_m'))}",
            f"Avg RSNR: {_format_num(rsnr.get('avg'), 1)}",
        ])
    elif selected:
        # Always show date at the top even when the basic flight-stat group is not selected.
        lines.append(f"Date: {escape(str(time_s.get('date', 'n/a')))}")

    autonomy = flight_stats.get("autonomy", {})
    if selected and autonomy.get("classification") == "semi_autonomous":
        modes = ", ".join(autonomy.get("autonomous_modes", [])) or "controller-managed mode(s)"
        lines.extend([
            "<b>Flight control</b>",
            f"Semi-autonomous time: {float(autonomy.get('autonomous_fraction', 0.0)) * 100.0:.1f}% ({escape(modes)})",
            "RC throttle is command input in these modes; it may not equal actual motor/TECS output.",
        ])

    if "signal" in selected:
        rssi_info = flight_stats.get("rssi", {})
        signal_title = "Signal (diversity)" if rssi_info.get("diversity") else "Signal"
        lines.append(f"<b>{signal_title}</b>")
        rqly = numeric.get("rqly", {})
        rsnr = numeric.get("rsnr", {})
        rssi = numeric.get("rssi_best", {})
        tpwr = numeric.get("tpwr", {})
        lines.extend([
            f"RQly avg/low: {_format_num(rqly.get('avg'), 0, '%')} / {_format_num(rqly.get('min'), 0, '%')}",
            f"RSNR avg/low/high: {_format_num(rsnr.get('avg'), 1)} / {_format_num(rsnr.get('min'), 1)} / {_format_num(rsnr.get('max'), 1)}",
            f"RSSI dBm avg/low/high: {_format_num(rssi.get('avg'), 1, ' dBm')} / {_format_num(rssi.get('min'), 1, ' dBm')} / {_format_num(rssi.get('max'), 1, ' dBm')}",
            f"TPWR avg/max: {_format_num(tpwr.get('avg'), 0, ' mW')} / {_format_num(tpwr.get('max'), 0, ' mW')}",
        ])

    if "battery" in selected:
        lines.append("<b>Battery / power</b>")
        curr = numeric.get("curr", {})
        power = numeric.get("power_w", {})
        gspd = numeric.get("gspd", {})
        if "default" not in selected:
            lines.extend([
                f"Avg speed: {_format_num(gspd.get('avg'), 1, ' km/h')}",
                f"Max speed: {_format_num(gspd.get('max'), 1, ' km/h')}",
            ])
        lines.extend([
            f"Current avg/max: {_format_num(curr.get('avg'), 1, ' A')} / {_format_num(curr.get('max'), 1, ' A')}",
            f"Power avg/max: {_format_num(power.get('avg'), 1, ' W')} / {_format_num(power.get('max'), 1, ' W')}",
            f"Capacity used: {_format_num(efficiency.get('capacity_used_mAh'), 0, ' mAh')}",
            f"Efficiency avg/max: {_format_num(efficiency.get('avg_mAh_per_km'), 0, ' mAh/km')} / {_format_num(efficiency.get('max_mAh_per_km'), 0, ' mAh/km')}",
        ])

    if "throttle" in selected:
        lines.append("<b>Throttle</b>")
        thr = numeric.get("throttle_pct", {})
        thr_us = numeric.get("throttle_us", {})
        lines.extend([
            f"Throttle avg/high: {_format_num(thr.get('avg'), 1, '%')} / {_format_num(thr.get('max'), 1, '%')}",
            f"Throttle us avg/high: {_format_num(thr_us.get('avg'), 0, ' us')} / {_format_num(thr_us.get('max'), 0, ' us')}",
        ])

    if "gps" in selected:
        lines.append("<b>GPS quality</b>")
        sats = numeric.get("sats", {})
        lines.extend([
            f"Sats avg/max: {_format_num(sats.get('avg'), 0)} / {_format_num(sats.get('max'), 0)}",
        ])
        warning = _four_sat_warning_text(flight_stats.get("parse", {}))
        if warning:
            lines.append(warning)

    if "altitude" in selected:
        lines.append("<b>Altitude</b>")
        alt = numeric.get("alt", {})
        if altitude_info.get("takeoff_elevation_m") is not None:
            lines.append(f"Takeoff elevation: {_format_num(altitude_info.get('takeoff_elevation_m'), 1, ' m')}")
        lines.extend([
            f"Rel. Alt avg/min/max: {_format_num(alt.get('avg'), 1, ' m')} / {_format_num(alt.get('min'), 1, ' m')} / {_format_num(alt.get('max'), 1, ' m')}",
            f"Time to max altitude: {_format_alt_time_to_max(altitude_info.get('time_to_max_s'), altitude_info.get('display', ''))}",
        ])

    if "agl_altitude" in selected:
        if "altitude" not in selected:
            lines.append("<b>Altitude</b>")
        agl = numeric.get("agl_alt", {})
        lines.append(
            f"AGL Alt avg/max: {_format_num(agl.get('avg'), 1, ' m')} / {_format_num(agl.get('max'), 1, ' m')}"
        )

    if "extra_telemetry" in selected:
        vspd = numeric.get("vspd_logged", {})
        temp = numeric.get("temperature", {})
        has_vspd = bool(vspd.get("count"))
        has_temp = bool(temp.get("count"))
        if has_vspd or has_temp:
            lines.append("<b>Additional telemetry</b>")
            if has_vspd:
                lines.append(
                    f"Logged VSpd avg/min/max: {_format_num(vspd.get('avg'), 2, ' m/s')} / {_format_num(vspd.get('min'), 2, ' m/s')} / {_format_num(vspd.get('max'), 2, ' m/s')}"
                )
            if has_temp:
                lines.append(
                    f"Temperature avg/min/max: {_format_num(temp.get('avg'), 1, ' °C')} / {_format_num(temp.get('min'), 1, ' °C')} / {_format_num(temp.get('max'), 1, ' °C')}"
                )

    warning = _four_sat_warning_text(flight_stats.get("parse", {}))
    if "gps" not in selected and warning:
        lines.append("<b>GPS quality</b>")
        lines.append(warning)

    if privacy_config.get("show_status"):
        if privacy_config.get("enabled"):
            start_m = float(privacy_config.get("start_meters", privacy_config.get("meters", 0.0)) or 0.0)
            end_m = float(privacy_config.get("end_meters", privacy_config.get("meters", 0.0)) or 0.0)
            if abs(start_m - end_m) < 0.001:
                lines.append(f"Privacy: first/last {_format_distance(start_m)} hidden")
            else:
                lines.append(f"Privacy: start {_format_distance(start_m)} hidden; end {_format_distance(end_m)} hidden")
        else:
            lines.append("Privacy: full GPS track shown")

    return lines


def _strip_html_for_console(line: str) -> str:
    """Make an HTML stats line readable in the console."""
    text_line = str(line)
    text_line = text_line.replace("<b>", "").replace("</b>", "")
    text_line = text_line.replace("<br>", " ")
    text_line = re.sub(r"<[^>]+>", "", text_line)
    return (
        text_line
        .replace("&amp;", "&")
        .replace("&lt;", "<")
        .replace("&gt;", ">")
        .replace("&quot;", '"')
        .replace("&#x27;", "'")
        .replace("&copy;", "(c)")
    )


def _parse_line_number_list(raw: str, max_line: int) -> Optional[List[int]]:
    """Parse comma/space-separated line numbers, e.g. '1 4 5' or '1,4,5'."""
    s = (raw or "").strip().lower()
    if not s:
        return []

    tokens = [tok for tok in re.split(r"[\s,;]+", s) if tok]
    if not tokens:
        return []

    nums: List[int] = []
    for tok in tokens:
        if not tok.isdigit():
            return None
        n = int(tok)
        if n < 1 or n > max_line:
            return None
        if n not in nums:
            nums.append(n)

    return nums


def _filter_lines_by_removed_indices(lines: List[str], removed_indices: Optional[List[int]]) -> List[str]:
    """Legacy helper: remove 1-based line numbers from a list of stats lines."""
    removed = set(removed_indices or [])
    return [line for i, line in enumerate(lines, start=1) if i not in removed]


def _filter_lines_by_removed_keys(lines: List[str], removed_keys: Optional[List[str]]) -> List[str]:
    """Remove stats lines by stable keys, preserving unlike structures in recursive batches."""
    removed = set(removed_keys or [])
    if not removed:
        return lines
    keys = _stat_line_keys(lines)
    return [line for line, key in zip(lines, keys) if key not in removed]


def _stats_lines_to_html(lines: List[str]) -> str:
    return "<br>".join(lines)



def _stat_line_keys(lines: List[str]) -> List[str]:
    """
    Create stable keys for stats lines so recursive batches can remove matching lines
    without relying on line numbers that may shift between Betaflight and ArduPilot logs.
    """
    keys: List[str] = []
    current_section = ""
    for line in lines:
        plain = _strip_html_for_console(line).strip()
        is_heading = str(line).strip().startswith("<b>") and str(line).strip().endswith("</b>")
        if is_heading:
            current_section = plain.lower()
            keys.append(f"heading|{current_section}")
            continue
        label = plain.split(":", 1)[0].strip().lower() if ":" in plain else plain.lower()
        keys.append(f"{current_section}|{label}")
    return keys


def choose_stats_lines_to_remove(lines: List[str], title: str = "Stats box preview") -> List[str]:
    """
    Show exact stats-box lines and let the user remove individual line numbers.
    Returns stable line keys rather than raw line numbers, so recursive batches can keep
    ArduPilot/Betaflight structure differences intact.
    """
    if not lines:
        return []

    line_keys = _stat_line_keys(lines)

    while True:
        print(f"\n{title}:")
        for i, line in enumerate(lines, start=1):
            print(f"{i}) {_strip_html_for_console(line)}")

        raw = input("Lines to remove from the stats box (Enter = keep all, or numbers like 2 5 6): ").strip()
        nums = _parse_line_number_list(raw, len(lines))
        if nums is None:
            print(f"❌ Input not recognized. Use line numbers from 1 to {len(lines)}, separated by spaces or commas.")
            continue

        if not nums:
            return []

        print("\nYou selected these line(s) to remove:")
        for n in nums:
            print(f"{n}) {_strip_html_for_console(lines[n - 1])}")

        if _ask_yes_no("Confirm removal?", default=True):
            return [line_keys[n - 1] for n in nums]

        print("Okay, let's choose again.")


def build_stats_lines(flight_stats: Dict[str, Any], stats_config: Dict[str, Any], privacy_config: Dict[str, Any]) -> List[str]:
    """Build stats lines and apply any user-selected line removals."""
    if not stats_config.get("enabled"):
        return []
    groups = stats_config.get("groups", ["default"])
    throttle_col_name = stats_config.get("throttle_channel", "CH3(us)")
    lines = _flight_stat_lines(flight_stats, groups, privacy_config, throttle_col_name)
    if stats_config.get("removed_line_keys"):
        return _filter_lines_by_removed_keys(lines, stats_config.get("removed_line_keys"))
    return _filter_lines_by_removed_indices(lines, stats_config.get("removed_line_indices"))


def finalize_run_options(run_options: Dict[str, Any], sample_csv_path: Optional[str], preview_csv_paths: Optional[List[str]] = None) -> Dict[str, Any]:
    """
    Final prompt before export: show exact stats lines and allow removal.

    For recursive mode, this looks for unique stats-line structures across the batch.
    Removed lines are saved by stable line keys, so ArduPilot-only lines like takeoff
    elevation are preserved unless the user specifically removes that key.
    """
    stats_config = run_options.get("stats_config", {})
    if not stats_config.get("enabled"):
        return run_options

    paths = [p for p in (preview_csv_paths or ([sample_csv_path] if sample_csv_path else [])) if p]
    if not paths:
        return run_options

    try:
        unique_structures: List[Tuple[str, List[str]]] = []
        seen_structures = set()
        for path in paths:
            effective_config = effective_stats_config_for_csv(stats_config, path)
            throttle_col_name = effective_config.get("throttle_channel", "CH3(us)")
            sample_flight_data = read_flight_data(path, min_sats=int(run_options.get("min_sats", MIN_SATS) or MIN_SATS), throttle_col_name=throttle_col_name)
            preview_lines = _flight_stat_lines(
                sample_flight_data.get("flight_stats", {}),
                effective_config.get("groups", ["default"]),
                run_options.get("privacy_config", {"enabled": False}),
                throttle_col_name,
            )
            structure_key = tuple(_stat_line_keys(preview_lines))
            if structure_key not in seen_structures:
                seen_structures.add(structure_key)
                unique_structures.append((os.path.basename(path), preview_lines))

        all_removed_keys: List[str] = []
        for idx, (filename, preview_lines) in enumerate(unique_structures, start=1):
            title = "Stats box preview before export"
            if len(unique_structures) > 1:
                title = f"Stats box preview before export — structure {idx} example: {filename}"
            removed_keys = choose_stats_lines_to_remove(preview_lines, title=title)
            for key in removed_keys:
                if key not in all_removed_keys:
                    all_removed_keys.append(key)

        stats_config = dict(stats_config)
        stats_config["removed_line_keys"] = all_removed_keys
        stats_config.pop("removed_line_indices", None)
        run_options = dict(run_options)
        run_options["stats_config"] = stats_config
    except Exception as exc:
        print(f"⚠️  Could not preview stats lines before export: {exc}")

    return run_options


def build_stats_html(flight_stats: Dict[str, Any], stats_config: Dict[str, Any], privacy_config: Dict[str, Any]) -> str:
    lines = build_stats_lines(flight_stats, stats_config, privacy_config)
    if not lines:
        return ""
    return _stats_lines_to_html(lines)


def apply_privacy_trim(
    segments: List[List[List[float]]],
    privacy_config: Dict[str, Any],
) -> Tuple[List[List[List[float]]], bool]:
    """
    Remove displayed points near the start and/or end of the map.

    Important privacy behavior:
    - Removed coordinates are not passed into the HTML generator.
    - If privacy trimming would remove too much track, this returns success=False so the caller can skip export.
    """
    if not privacy_config.get("enabled"):
        return segments, True

    start_m = float(privacy_config.get("start_meters", privacy_config.get("meters", 0.0)) or 0.0)
    end_m = float(privacy_config.get("end_meters", privacy_config.get("meters", 0.0)) or 0.0)
    if start_m <= 0 and end_m <= 0:
        return segments, True

    all_points = [pt for seg in segments for pt in seg]
    if len(all_points) < 2:
        return [], False

    home = all_points[0]
    landing = all_points[-1]

    start_trimmed: List[List[List[float]]] = []
    started = start_m <= 0
    for seg in segments:
        new_seg: List[List[float]] = []
        for pt in seg:
            if not started:
                if haversine_m(home, pt) < start_m:
                    continue
                started = True
            new_seg.append(pt)
        if new_seg:
            start_trimmed.append(new_seg)

    end_trimmed_reversed: List[List[List[float]]] = []
    ended = end_m <= 0
    for seg in reversed(start_trimmed):
        new_seg_rev: List[List[float]] = []
        for pt in reversed(seg):
            if not ended:
                if haversine_m(landing, pt) < end_m:
                    continue
                ended = True
            new_seg_rev.append(pt)
        new_seg = list(reversed(new_seg_rev))
        if new_seg:
            end_trimmed_reversed.append(new_seg)

    trimmed = list(reversed(end_trimmed_reversed))
    trimmed_points = [pt for seg in trimmed for pt in seg]
    if len(trimmed_points) < 2:
        return [], False

    return trimmed, True


def apply_privacy_trim_heatmap(
    segments: List[List[Dict[str, Any]]],
    privacy_config: Dict[str, Any],
) -> Tuple[List[List[Dict[str, Any]]], bool]:
    """Privacy-trim heatmap segments. Removed heatmap points are not written into the HTML."""
    if not privacy_config.get("enabled"):
        return segments, True

    start_m = float(privacy_config.get("start_meters", privacy_config.get("meters", 0.0)) or 0.0)
    end_m = float(privacy_config.get("end_meters", privacy_config.get("meters", 0.0)) or 0.0)
    if start_m <= 0 and end_m <= 0:
        return segments, True

    all_points = [p for seg in segments for p in seg]
    if len(all_points) < 2:
        return [], False

    home = [float(all_points[0]["lat"]), float(all_points[0]["lon"])]
    landing = [float(all_points[-1]["lat"]), float(all_points[-1]["lon"])]

    start_trimmed: List[List[Dict[str, Any]]] = []
    started = start_m <= 0
    for seg in segments:
        new_seg: List[Dict[str, Any]] = []
        for p in seg:
            pt = [float(p["lat"]), float(p["lon"])]
            if not started:
                if haversine_m(home, pt) < start_m:
                    continue
                started = True
            new_seg.append(p)
        if new_seg:
            start_trimmed.append(new_seg)

    end_trimmed_reversed: List[List[Dict[str, Any]]] = []
    ended = end_m <= 0
    for seg in reversed(start_trimmed):
        new_seg_rev: List[Dict[str, Any]] = []
        for p in reversed(seg):
            pt = [float(p["lat"]), float(p["lon"])]
            if not ended:
                if haversine_m(landing, pt) < end_m:
                    continue
                ended = True
            new_seg_rev.append(p)
        new_seg = list(reversed(new_seg_rev))
        if new_seg:
            end_trimmed_reversed.append(new_seg)

    trimmed = list(reversed(end_trimmed_reversed))
    if sum(len(seg) for seg in trimmed) < 2:
        return [], False

    return trimmed, True


def _tile_js_single(map_var: str, tile_key: str) -> str:
    """JavaScript for a single basemap."""
    tile = TILE_PROVIDERS.get(tile_key, TILE_PROVIDERS[DEFAULT_TILE_KEY])
    tile_url_json = json.dumps(tile["url"])
    tile_options_json = json.dumps(tile["options"])
    return f"""
  var tile = L.tileLayer(
      {tile_url_json},
      {tile_options_json}
  ).addTo({map_var});
"""


def _tile_js_layers(map_var: str, tile_keys: List[str]) -> str:
    """JavaScript for all selected basemaps as switchable Leaflet base layers."""
    lines: List[str] = ["\n  var baseLayers = {};"]
    first_layer_var = None

    for tile_key in tile_keys:
        tile = TILE_PROVIDERS.get(tile_key, TILE_PROVIDERS[DEFAULT_TILE_KEY])
        layer_var = f"tileLayer_{tile_key}"
        if first_layer_var is None:
            first_layer_var = layer_var
        layer_name = tile["name"].split(" (")[0]  # shorter layer-control label
        lines.append(f"""
  var {layer_var} = L.tileLayer(
      {json.dumps(tile["url"])},
      {json.dumps(tile["options"])}
  );
  baseLayers[{json.dumps(layer_name)}] = {layer_var};
""")

    if first_layer_var:
        lines.append(f"  {first_layer_var}.addTo({map_var});")
    lines.append(f"""
  L.control.layers(baseLayers, null, {{
      collapsed: true,
      position: "topright"
  }}).addTo({map_var});
""")
    return "\n".join(lines)


def _stats_control_js(map_var: str, stats_html: str, position: str) -> str:
    if not stats_html:
        return ""

    pos = "topleft" if position == "topleft" else "topright"
    stats_json = json.dumps(stats_html)

    return f"""
  var statsControl = L.control({{position: "{pos}"}});
  statsControl.onAdd = function(map) {{
      var div = L.DomUtil.create("div", "leaflet-control-attribution flight-stats-control");
      div.innerHTML = {stats_json};
      L.DomEvent.disableClickPropagation(div);
      return div;
  }};
  statsControl.addTo({map_var});
"""


def build_leaflet_html(
    segments: List[List[List[float]]],
    color_css: str,
    tile_key: str = DEFAULT_TILE_KEY,
    tile_keys: Optional[List[str]] = None,
    initial_tile_key: str = DEFAULT_TILE_KEY,
    use_layer_control: bool = False,
    stats_html: str = "",
    stats_position: str = "topright",
    zoom: int = DEFAULT_ZOOM,
    title: str = "Flight Path Map",
) -> str:
    """
    Draw discontinuous track:
    segments -> Leaflet "multi-polyline" (array of arrays).
    """
    all_points: List[List[float]] = [pt for seg in segments for pt in seg]
    if not all_points:
        all_points = [[0.0, 0.0]]

    lats = [p[0] for p in all_points]
    lons = [p[1] for p in all_points]
    center = [sum(lats) / len(lats), sum(lons) / len(lons)]

    start_pt = all_points[0]
    end_pt = all_points[-1]

    map_id = "map_" + uuid.uuid4().hex
    div_id = map_id

    segments_json = json.dumps(segments, separators=(",", ":"))

    if use_layer_control:
        layer_keys = _ordered_tile_keys(initial_tile_key, tile_keys if tile_keys else ALL_TILE_KEYS)
        tile_js = _tile_js_layers(map_id, layer_keys)
    else:
        tile_js = _tile_js_single(map_id, tile_key)

    stats_js = _stats_control_js(map_id, stats_html, stats_position)

    return f"""<!DOCTYPE html>
<html>
<head>
  <meta http-equiv="content-type" content="text/html; charset=UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no" />
  <meta name="referrer" content="no-referrer-when-downgrade" />
  <title>{escape(title)}</title>

  <script>
    L_NO_TOUCH = false;
    L_DISABLE_3D = false;
  </script>

  <style>html, body {{width: 100%;height: 100%;margin: 0;padding: 0;}}</style>
  <style>#{div_id} {{position:absolute;top:0;bottom:0;right:0;left:0;}}</style>
  <style>
    .leaflet-center {{
      left: 50%;
      transform: translateX(-50%);
    }}
    .flight-stats-control {{
      text-align: left;
      max-width: 380px;
      white-space: normal;
      line-height: 1.35;
      padding: 3px 6px;
      overflow: visible;
    }}
    .flight-stats-control b {{
      display: inline-block;
      margin-top: 2px;
      margin-bottom: 1px;
    }}
  </style>

  <script src="https://cdn.jsdelivr.net/npm/leaflet@1.6.0/dist/leaflet.js"></script>
  <script src="https://code.jquery.com/jquery-1.12.4.min.js"></script>
  <script src="https://maxcdn.bootstrapcdn.com/bootstrap/3.2.0/js/bootstrap.min.js"></script>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/Leaflet.awesome-markers/2.0.2/leaflet.awesome-markers.js"></script>

  <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/leaflet@1.6.0/dist/leaflet.css"/>
  <link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/bootstrap/3.2.0/css/bootstrap.min.css"/>
  <link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/bootstrap/3.2.0/css/bootstrap-theme.min.css"/>
  <link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/font-awesome/4.6.3/css/font-awesome.min.css"/>
  <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/Leaflet.awesome-markers/2.0.2/leaflet.awesome-markers.css"/>
  <link rel="stylesheet" href="https://cdn.jsdelivr.net/gh/python-visualization/folium/folium/templates/leaflet.awesome.rotate.min.css"/>
</head>
<body>
  <div class="folium-map" id="{div_id}"></div>
</body>
<script>
  var {map_id} = L.map("{div_id}", {{
      center: {json.dumps(center)},
      crs: L.CRS.EPSG3857,
      zoom: {zoom},
      zoomControl: true,
      preferCanvas: false
  }});
{tile_js}
  L.control.scale({{
      position: "bottomleft",
      metric: true,
      imperial: false,
      maxWidth: 200
  }}).addTo({map_id});
{stats_js}
  var segments = {segments_json};

  var poly = L.polyline(segments, {{
      bubblingMouseEvents: true,
      color: "{color_css}",
      opacity: 1.0,
      weight: 4
  }}).addTo({map_id});

  var start = {json.dumps(start_pt)};
  var end = {json.dumps(end_pt)};
  L.marker(start).addTo({map_id}).bindPopup("Start");
  L.marker(end).addTo({map_id}).bindPopup("End");

  {map_id}.fitBounds(poly.getBounds(), {{padding:[20,20]}});
</script>
</html>
"""



def _column_values_from_csv_sample(csv_path: str, max_rows: int = 500) -> Tuple[List[str], Dict[str, int]]:
    """Return header and a count of parseable numeric values per column from a sample of rows."""
    counts: Dict[str, int] = {}
    header: List[str] = []
    try:
        with open(csv_path, "r", encoding="utf-8-sig", errors="replace", newline="") as f:
            dialect = sniff_dialect(f)
            reader = csv.reader(f, dialect)
            header = next(reader)
            counts = {name: 0 for name in header}
            for row_num, row in enumerate(reader):
                if row_num >= max_rows:
                    break
                for idx, name in enumerate(header):
                    if name.lower() in ("date", "time", "gps"):
                        continue
                    if _parse_float(_clean_cell(row, idx)) is not None:
                        counts[name] += 1
    except Exception:
        pass
    return header, counts


def _analysis_col_key(name: str) -> str:
    return (name or "").strip().lower().replace(" ", "")


def _is_ignored_analysis_column(name: str) -> bool:
    """Filter raw data-analysis choices that are unhelpful/noisy for FPV analysis."""
    key = _analysis_col_key(name)
    if key in ("date", "time", "gps"):
        return True

    # EdgeTX transmitter/controller battery voltage, not the aircraft.
    if key.startswith("txbat") or key.startswith("txbt"):
        return True

    # RSSI is offered as one combined best-of-1RSS/2RSS option instead.
    if key.startswith("1rss") or key.startswith("2rss"):
        return True

    # CH17-CH32 are rarely useful for FPV flight review and clutter the menu.
    ch = re_match_channel(name.strip())
    if ch is not None and ch >= 17:
        return True

    return False


def _infer_analysis_unit(name: str) -> str:
    key = (name or "").lower()
    unit_patterns = [
        ("km/h", ("kmh", "km/h")), ("mAh", ("mah",)), ("mW", ("mw",)),
        ("dBm", ("dbm",)), ("dB", ("db",)), ("%", ("(%)", "%")),
        ("V", ("(v)", "volt")), ("A", ("(a)", "curr")), ("m", ("(m)", "alt")),
        ("µs", ("(us)",)), ("°", ("hdg", "heading", "(°)")),
    ]
    for unit, needles in unit_patterns:
        if any(n in key for n in needles):
            return unit
    return "value"


def _analysis_profile_for_name(name: str, higher_is_better: bool = True) -> str:
    key = (name or "").lower()
    if any(x in key for x in ("sats", "rqly", "quality", "rsnr", "rssi")):
        return "higher"
    if any(x in key for x in ("efficiency", "error", "residual")):
        return "lower"
    if any(x in key for x in ("speed", "power", "current", "alt", "distance", "capacity", "energy", "throttle", "turn", "accel", "heading")):
        return "bands"
    return "higher" if higher_is_better else "lower"


def _analysis_metric_options(csv_path: str) -> List[Dict[str, Any]]:
    """Build the integrated analysis menu from computed and useful raw CSV values."""
    header, counts = _column_values_from_csv_sample(csv_path)
    if not header:
        return []

    gps_idx = _find_col_index(header, "GPS")
    time_idx = _find_col_index(header, "Time")
    rssi1_idx = _find_any_col_index(header, ["1RSS", "1RSS(dBm)", "1RSS(dB)"])
    rssi2_idx = _find_any_col_index(header, ["2RSS", "2RSS(dBm)", "2RSS(dB)"])
    curr_idx = _find_any_col_index(header, ["Curr", "Curr(A)"])
    rxbt_idx = _find_any_col_index(header, ["RxBt", "RxBt(V)"])
    capa_idx = _find_any_col_index(header, ["Capa", "Capa(mAh)"])
    alt_idx = _find_any_col_index(header, ["Alt", "Alt(m)", "alt (m)"])
    gspd_idx = _find_any_col_index(header, ["GSpd", "GSpd(kmh)", "GSpd(km/h)"])
    ch3_idx = _find_any_col_index(header, ["CH3(us)", "CH3"])
    sats_idx = _find_col_index(header, "Sats")

    options: List[Dict[str, Any]] = []

    def add_computed(metric_id: str, label: str, short: str, unit: str, profile: str = "bands", higher_is_better: bool = True) -> None:
        options.append({
            "type": "computed", "id": metric_id, "label": label, "short": short,
            "unit": unit, "profile": profile, "higher_is_better": higher_is_better,
        })

    if sats_idx is not None:
        add_computed("satellites", "Satellite count", "Satellites", "sats", "higher", True)
    if rssi1_idx is not None or rssi2_idx is not None:
        add_computed("rssi_best", "RSSI dBm (best usable value from 1RSS/2RSS)", "RSSI_dBm", "dBm", "higher", True)
    if gps_idx is not None and time_idx is not None:
        add_computed("coordinate_speed_kmh", "Coordinate-derived speed over 2 seconds", "Coordinate_speed", "km/h", "bands", False)
    if curr_idx is not None and rxbt_idx is not None:
        add_computed("power_w", "Power (W) = current × RxBt", "Power_W", "W", "bands", False)
        add_computed("energy_used_Wh", "Integrated energy used", "Energy_used_Wh", "Wh", "bands", False)
    if gps_idx is not None and capa_idx is not None:
        add_computed("efficiency_mAh_km", "Capacity efficiency (mAh/km)", "Efficiency_mAh_km", "mAh/km", "lower", False)
    if gps_idx is not None and curr_idx is not None and rxbt_idx is not None:
        add_computed("energy_rate_Wh_km", "Energy efficiency (Wh/km)", "Efficiency_Wh_km", "Wh/km", "lower", False)
    if gps_idx is not None:
        add_computed("distance_home_m", "Distance from home", "Distance_from_home", "m", "bands", False)
        add_computed("cumulative_distance_km", "Cumulative distance", "Cumulative_distance", "km", "bands", False)
    if alt_idx is not None:
        add_computed("relative_alt_m", "Converted relative altitude", "Relative_altitude", "m", "bands", False)
    if alt_idx is not None and gps_idx is not None:
        add_computed("altitude_msl", "Altitude MSL (terrain takeoff reference)", "Altitude_MSL", "m", "bands", False)
        add_computed("terrain_msl", "Terrain elevation", "Terrain_elevation", "m", "bands", False)
        add_computed("altitude_agl", "Altitude AGL above current terrain", "Altitude_AGL", "m", "bands", False)
    if alt_idx is not None and time_idx is not None:
        add_computed("climb_rate_ms", "Vertical speed (most accurate with barometric altitude)", "Vertical_speed", "m/s", "bands", True)
    if gspd_idx is not None and time_idx is not None:
        add_computed("acceleration_mps2", "Acceleration from logged speed", "Acceleration", "m/s²", "bands", True)
    if gps_idx is not None and time_idx is not None:
        add_computed("turn_rate_deg_s", "Turn rate from GPS course", "Turn_rate", "°/s", "bands", True)
    if ch3_idx is not None:
        add_computed("throttle_pct_ch3", "Throttle % from CH3(us)", "Throttle_pct_CH3", "%", "bands", False)
    if capa_idx is not None:
        add_computed("capacity_used_mAh", "Capacity used during this CSV", "Capacity_used_mAh", "mAh", "bands", False)

    # Resolve exact duplicate raw sensor names by content rather than source order.
    # Older CRSF/ArduPilot EdgeTX logs can contain two identically named Ptch columns,
    # one of which is a constant placeholder.  Expose one semantic raw parameter and
    # point it at the populated/changing candidate so analysis does not silently read
    # the wrong duplicate.
    sample_rows: List[List[str]] = []
    try:
        with open(csv_path, "r", encoding="utf-8-sig", errors="replace", newline="") as f:
            dialect = sniff_dialect(f)
            reader = csv.reader(f, dialect)
            _ = next(reader, None)
            for n, row in enumerate(reader):
                if n >= 5000:
                    break
                sample_rows.append(list(row))
    except Exception:
        sample_rows = []

    seen_raw_names: set[str] = set()
    for source_index, name in enumerate(header):
        norm = _normalise_header_name(name)
        if norm in seen_raw_names:
            continue
        seen_raw_names.add(norm)
        if counts.get(name, 0) <= 0 or _is_ignored_analysis_column(name):
            continue
        candidates = [i for i, candidate_name in enumerate(header) if _normalise_header_name(candidate_name) == norm]
        chosen_index = source_index
        if len(candidates) > 1 and sample_rows:
            chosen_index = max(candidates, key=lambda i: (_numeric_column_profile(sample_rows, i)["score"], i))
        unit = _infer_analysis_unit(name)
        profile = _analysis_profile_for_name(name, True)
        options.append({
            "type": "raw", "id": f"raw::{name}", "label": name,
            "short": "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in name).strip("_") or "value",
            "column": name, "column_index": chosen_index, "unit": unit, "profile": profile,
            "higher_is_better": profile == "higher",
        })
    return options

def choose_heatmap_stat_column(csv_path: str) -> Dict[str, Any]:
    """Ask the user which numeric/computed value to use for data-analysis colouring."""
    options = _analysis_metric_options(csv_path)

    if not options:
        raise ValueError("No numeric telemetry columns were found for data analysis.")

    print("\nChoose a value to colour the GPS track by:")
    print("Computed options are listed first, followed by useful raw CSV columns.")
    for i, option in enumerate(options, start=1):
        print(f"{i}) {option['label']}")

    while True:
        raw = input("Value choice (type a number or exact name): ").strip()
        if not raw:
            print("❌ Please choose a listed number or exact value name.")
            continue

        try:
            idx = int(raw)
            if 1 <= idx <= len(options):
                return options[idx - 1]
        except Exception:
            pass

        raw_lower = raw.lower()
        for option in options:
            if raw_lower in (str(option["label"]).lower(), str(option["id"]).lower(), str(option["short"]).lower()):
                return option

        print("❌ Value not recognized. Use a listed number or the exact value name.")


def _normalise_hex_colour(hex_color: str) -> str:
    h = (hex_color or "").strip().lower()
    if not h.startswith("#"):
        h = "#" + h
    if len(h) == 4:
        h = "#" + "".join(ch * 2 for ch in h[1:])
    return h


def _parse_heatmap_color_choice(raw: str, default_hex: str) -> Dict[str, str]:
    """
    Parse a heatmap colour and keep a friendly label.

    If the user typed a colour name/letter, the label uses the colour name.
    If the user pressed Enter or typed a hex code, the label uses the hex code.
    """
    s = (raw or "").strip().lower()
    default_hex = _normalise_hex_colour(default_hex)

    if not s:
        default_names = {
            "#ff0000": "red",
            "#00aa00": "green",
            "#0000ff": "blue",
            "#ffff00": "yellow",
            "#ffa500": "orange",
            "#4b0082": "indigo",
            "#8a2be2": "violet",
        }
        return {"hex": default_hex, "label": default_names.get(default_hex, default_hex)}

    roygbiv = {
        "r": ("red", "#ff0000"), "red": ("red", "#ff0000"),
        "o": ("orange", "#ffa500"), "orange": ("orange", "#ffa500"),
        "y": ("yellow", "#ffff00"), "yellow": ("yellow", "#ffff00"),
        "g": ("green", "#00aa00"), "green": ("green", "#00aa00"),
        "b": ("blue", "#0000ff"), "blue": ("blue", "#0000ff"),
        "i": ("indigo", "#4b0082"), "indigo": ("indigo", "#4b0082"),
        "v": ("violet", "#8a2be2"), "violet": ("violet", "#8a2be2"), "purple": ("purple", "#8a2be2"),
    }
    if s in roygbiv:
        name, hex_value = roygbiv[s]
        return {"hex": hex_value, "label": name}

    common = {
        "black": "#000000", "white": "#ffffff", "gray": "#808080", "grey": "#808080",
        "silver": "#c0c0c0", "maroon": "#800000", "fuchsia": "#ff00ff",
        "lime": "#00ff00", "olive": "#808000", "navy": "#000080", "teal": "#008080",
        "aqua": "#00ffff", "cyan": "#00ffff", "magenta": "#ff00ff", "brown": "#a52a2a",
        "pink": "#ffc0cb", "gold": "#ffd700", "coral": "#ff7f50", "turquoise": "#40e0d0",
    }
    if s in common:
        return {"hex": common[s], "label": s}

    if s.startswith("#"):
        hexpart = s[1:]
        if len(hexpart) in (3, 6) and all(c in "0123456789abcdef" for c in hexpart):
            hex_value = _normalise_hex_colour(s)
            return {"hex": hex_value, "label": hex_value}

    return {"hex": "", "label": ""}


def choose_heatmap_style(default_higher_is_better: bool = True) -> Dict[str, Any]:
    """Choose two-colour gradient or single-colour opacity mode for data analysis."""
    while True:
        print("\nData-analysis colour mode:")
        print("1) Two-colour gradient: choose colours for worst and best values [default]")
        print("2) Single colour with opacity: better values are more opaque")
        raw = input("Choose colour mode (Enter/1 or 2): ").strip().lower()

        if raw in ("", "1", "gradient", "two", "g"):
            while True:
                worst = input("Colour for worst values (type ROYGBIV or hex code) [default red]: ").strip()
                worst_info = _parse_heatmap_color_choice(worst, "#ff0000")
                if worst_info["hex"]:
                    break
                print("❌ Colour not recognized. Use ROYGBIV words/letters like red/r, or a hex code like #ff0000.")
            while True:
                best = input("Colour for best values (type ROYGBIV or hex code) [default green]: ").strip()
                best_info = _parse_heatmap_color_choice(best, "#00aa00")
                if best_info["hex"]:
                    break
                print("❌ Colour not recognized. Use ROYGBIV words/letters like green/g, or a hex code like #00aa00.")

            higher_is_better = _ask_yes_no("Should higher values be treated as better?", default=default_higher_is_better)
            return {
                "mode": "gradient",
                "worst_color": worst_info["hex"],
                "worst_label": worst_info["label"],
                "best_color": best_info["hex"],
                "best_label": best_info["label"],
                "higher_is_better": higher_is_better,
            }

        if raw in ("2", "opacity", "single", "s"):
            while True:
                base = input("Base colour for the heatmap track (type ROYGBIV or hex code) [default blue]: ").strip()
                base_info = _parse_heatmap_color_choice(base, "#0000ff")
                if base_info["hex"]:
                    break
                print("❌ Colour not recognized. Use ROYGBIV words/letters like blue/b/yellow/y, or a hex code like #00aaff.")

            higher_is_better = _ask_yes_no("Should higher values be treated as better?", default=default_higher_is_better)
            return {
                "mode": "opacity",
                "base_color": base_info["hex"],
                "base_label": base_info["label"],
                "higher_is_better": higher_is_better,
            }

        print("❌ Colour mode not recognized. Accepted inputs: Enter/1/gradient or 2/opacity.")


def _hex_to_rgb(hex_color: str) -> Tuple[int, int, int]:
    c = hex_color.strip().lstrip("#")
    return int(c[0:2], 16), int(c[2:4], 16), int(c[4:6], 16)


def _rgb_to_hex(rgb: Tuple[int, int, int]) -> str:
    return "#{:02x}{:02x}{:02x}".format(*rgb)


def _interpolate_hex(a: str, b: str, t: float) -> str:
    t = max(0.0, min(1.0, t))
    ar, ag, ab = _hex_to_rgb(a)
    br, bg, bb = _hex_to_rgb(b)
    return _rgb_to_hex((round(ar + (br - ar) * t), round(ag + (bg - ag) * t), round(ab + (bb - ab) * t)))


def _relative_alt_value_for_heatmap(alt_value: Optional[float], ctx: Dict[str, Any]) -> Optional[float]:
    """Return relative altitude for heatmap computed metrics."""
    if alt_value is None:
        return None

    if ctx.get("alt_first") is None:
        ctx["alt_first"] = alt_value
        ctx["bf_alt_waiting_for_reset"] = abs(float(alt_value)) >= ALT_MSL_LOOKING_THRESHOLD_M and ctx.get("alt_source") != "ardupilot_asl"

    if _altitude_source_is_asl(ctx.get("alt_source")):
        return float(alt_value) - float(ctx.get("alt_first") or 0.0)

    if ctx.get("bf_alt_waiting_for_reset"):
        if abs(float(alt_value)) <= ALT_RELATIVE_ZERO_THRESHOLD_M:
            ctx["bf_alt_waiting_for_reset"] = False
        else:
            return None

    return float(alt_value)


def _metric_value_from_row(
    row: List[str],
    metric: Dict[str, Any],
    idx: Dict[str, Optional[int]],
    ctx: Dict[str, Any],
    gps_point: List[float],
) -> Optional[float]:
    """Compute or read the selected heatmap value for one CSV row."""
    if metric.get("type") == "raw":
        return _parse_float(_clean_cell(row, idx.get("raw_value")))

    metric_id = metric.get("id")

    if metric_id == "rssi_best":
        r1 = _parse_float(_clean_cell(row, idx.get("rssi1")))
        r2 = _parse_float(_clean_cell(row, idx.get("rssi2")))
        return _best_rssi_from_values(r1, r2)

    curr = _parse_float(_clean_cell(row, idx.get("curr")))
    rxbt = _parse_float(_clean_cell(row, idx.get("rxbt")))
    capa = _parse_float(_clean_cell(row, idx.get("capa")))
    gspd = _parse_float(_clean_cell(row, idx.get("gspd")))
    alt_raw = _parse_float(_clean_cell(row, idx.get("alt")))
    time_value = _parse_datetime_value(_clean_cell(row, idx.get("date")), _clean_cell(row, idx.get("time")))

    # Update distance context for computed distance/efficiency metrics.
    if ctx.get("home_point") is None:
        ctx["home_point"] = gps_point
    if ctx.get("prev_point") is not None:
        ctx["cum_distance_m"] = float(ctx.get("cum_distance_m", 0.0)) + haversine_m(ctx["prev_point"], gps_point)
    ctx["prev_point"] = gps_point

    if metric_id == "power_w":
        if curr is None or rxbt is None:
            return None
        return curr * rxbt

    if metric_id == "distance_home_m":
        return haversine_m(ctx["home_point"], gps_point)

    if metric_id == "cumulative_distance_km":
        return float(ctx.get("cum_distance_m", 0.0)) / 1000.0

    if metric_id == "capacity_used_mAh":
        if capa is None:
            return None
        if ctx.get("start_capa") is None:
            ctx["start_capa"] = capa
        return max(0.0, capa - float(ctx["start_capa"]))

    if metric_id == "efficiency_mAh_km":
        if capa is None:
            return None
        if ctx.get("start_capa") is None:
            ctx["start_capa"] = capa
        if float(ctx.get("cum_distance_m", 0.0)) < 100.0:
            return None
        capacity_used = max(0.0, capa - float(ctx["start_capa"]))
        return capacity_used / (float(ctx.get("cum_distance_m", 0.0)) / 1000.0)

    if metric_id == "energy_rate_Wh_km":
        if curr is None or rxbt is None or gspd is None or gspd <= 1.0:
            return None
        return (curr * rxbt) / gspd

    if metric_id == "relative_alt_m":
        return _relative_alt_value_for_heatmap(alt_raw, ctx)

    if metric_id == "climb_rate_ms":
        rel_alt = _relative_alt_value_for_heatmap(alt_raw, ctx)
        if rel_alt is None or time_value is None:
            return None
        prev_alt = ctx.get("prev_alt_for_rate")
        prev_time = ctx.get("prev_time_for_rate")
        ctx["prev_alt_for_rate"] = rel_alt
        ctx["prev_time_for_rate"] = time_value
        if prev_alt is None or prev_time is None:
            return None
        dt = float(time_value) - float(prev_time)
        if dt <= 0:
            return None
        return (float(rel_alt) - float(prev_alt)) / dt

    if metric_id == "throttle_pct_ch3":
        ch3 = _parse_float(_clean_cell(row, idx.get("ch3")))
        if ch3 is None:
            return None
        throttle = (ch3 - THROTTLE_MIN_US) / (THROTTLE_MAX_US - THROTTLE_MIN_US) * 100.0
        return max(0.0, min(100.0, throttle))

    return None


def read_heatmap_segments(csv_path: str, metric: Dict[str, Any]) -> Tuple[List[List[Dict[str, Any]]], Dict[str, Any]]:
    """
    Read GPS track segments with one numeric/computed value per point for data-analysis maps.
    Track continuity follows the same GPS and sats rules as the regular map.
    """
    segments: List[List[Dict[str, Any]]] = []
    current: List[Dict[str, Any]] = []
    rows = 0
    bad_gps = 0
    missing_gps = 0
    low_sats = 0
    missing_value = 0
    deduped = 0
    values: List[float] = []
    last_key: Optional[Tuple[float, float, float]] = None

    def close_segment():
        nonlocal current, last_key
        if current:
            segments.append(current)
        current = []
        last_key = None

    with open(csv_path, "r", encoding="utf-8-sig", errors="replace", newline="") as f:
        dialect = sniff_dialect(f)
        reader = csv.reader(f, dialect)

        try:
            header = next(reader)
        except StopIteration:
            return [], {"rows": 0, "values": 0}

        # Read the rows before resolving columns so duplicate sensor names and the
        # newer second combined GPS/UTC ``Date`` field can be disambiguated by content.
        data_rows = [list(row) for row in reader]
        datetime_idx = _select_datetime_columns(header, data_rows)

        raw_value_idx: Optional[int] = None
        if metric.get("type") == "raw":
            try:
                candidate = int(metric.get("column_index")) if metric.get("column_index") is not None else None
            except Exception:
                candidate = None
            if candidate is not None and 0 <= candidate < len(header):
                raw_value_idx = candidate
            else:
                raw_value_idx = _best_numeric_col_index(header, data_rows, [str(metric.get("column", ""))])

        idx = {
            "gps": _find_col_index(header, "GPS"),
            "sats": _best_numeric_col_index(header, data_rows, ["sats"]),
            "date": datetime_idx.get("date"),
            "time": datetime_idx.get("time"),
            "utc_datetime": datetime_idx.get("utc_datetime"),
            "raw_value": raw_value_idx,
            "rssi1": _best_numeric_col_index(header, data_rows, ["1RSS", "1RSS(dBm)", "1RSS(dB)"]),
            "rssi2": _best_numeric_col_index(header, data_rows, ["2RSS", "2RSS(dBm)", "2RSS(dB)"]),
            "curr": _best_numeric_col_index(header, data_rows, ["Curr", "Curr(A)"]),
            "rxbt": _best_numeric_col_index(header, data_rows, ["RxBt", "RxBt(V)"]),
            "capa": _best_numeric_col_index(header, data_rows, ["Capa", "Capa(mAh)"]),
            "alt": _best_numeric_col_index(header, data_rows, ["Alt", "Alt(m)", "alt (m)"]),
            "gspd": _best_numeric_col_index(header, data_rows, ["GSpd", "GSpd(kmh)", "GSpd(km/h)"]),
            "ch3": _best_numeric_col_index(header, data_rows, ["CH3(us)", "CH3"]),
        }

        if idx["gps"] is None:
            return [], {"rows": 0, "values": 0}

        ctx: Dict[str, Any] = {
            "home_point": None,
            "prev_point": None,
            "cum_distance_m": 0.0,
            "start_capa": None,
            "alt_source": detect_altitude_source_csv(csv_path),
            "alt_first": None,
            "bf_alt_waiting_for_reset": False,
            "prev_alt_for_rate": None,
            "prev_time_for_rate": None,
        }

        for row in data_rows:
            rows += 1

            sats_ok = True
            if idx["sats"] is not None:
                sats_val = _parse_sats(_clean_cell(row, idx["sats"]))
                if sats_val is None or sats_val < float(MIN_SATS):
                    sats_ok = False

            gps_parsed = _parse_gps_cell(_clean_cell(row, idx["gps"]))
            if gps_parsed is None:
                close_segment()
                if not _clean_cell(row, idx["gps"]):
                    missing_gps += 1
                else:
                    bad_gps += 1
                continue

            if not sats_ok:
                close_segment()
                low_sats += 1
                continue

            lat, lon = gps_parsed
            gps_point = [round(lat, DEDUP_DECIMALS), round(lon, DEDUP_DECIMALS)]

            value = _metric_value_from_row(row, metric, idx, ctx, gps_point)
            if value is None:
                close_segment()
                missing_value += 1
                continue

            key = (gps_point[0], gps_point[1], round(float(value), 6))
            if last_key == key:
                deduped += 1
                continue
            last_key = key

            point = {"lat": key[0], "lon": key[1], "value": float(value)}
            current.append(point)
            values.append(float(value))

    close_segment()

    return segments, {
        "rows": rows,
        "values": len(values),
        "min": min(values) if values else None,
        "max": max(values) if values else None,
        "bad_gps": bad_gps,
        "missing_gps": missing_gps,
        "low_sats": low_sats,
        "missing_value": missing_value,
        "deduped": deduped,
    }


def _heatmap_segment_js(heatmap_segments: List[List[Dict[str, Any]]], style: Dict[str, Any], value_min: float, value_max: float) -> str:
    """Create JS that draws one styled polyline for every point-to-point segment."""
    segment_lines: List[Dict[str, Any]] = []
    higher_is_better = bool(style.get("higher_is_better", True))
    denom = value_max - value_min if value_max != value_min else 1.0

    for seg in heatmap_segments:
        if len(seg) < 2:
            continue
        for a, b in zip(seg, seg[1:]):
            avg_value = (float(a["value"]) + float(b["value"])) / 2.0
            t = (avg_value - value_min) / denom
            t = max(0.0, min(1.0, t))
            quality = t if higher_is_better else 1.0 - t

            if style.get("mode") == "gradient":
                color = _interpolate_hex(style["worst_color"], style["best_color"], quality)
                opacity = 1.0
            else:
                color = style.get("base_color", "#0000ff")
                opacity = 0.20 + (0.80 * quality)

            segment_lines.append({
                "coords": [[a["lat"], a["lon"]], [b["lat"], b["lon"]]],
                "color": color,
                "opacity": round(opacity, 3),
                "value": round(avg_value, 3),
            })

    return json.dumps(segment_lines, separators=(",", ":"))


def _data_analysis_info_lines(metric: Dict[str, Any], style: Dict[str, Any], value_min: float, value_max: float) -> List[str]:
    """Build the Data analysis section added to the normal stats box."""
    if style.get("mode") == "gradient":
        colour_description = f"Worst values use {style['worst_label']}; best values use {style['best_label']}."
    else:
        colour_description = f"Worst values are faint; best values are most opaque using {style.get('base_label', style.get('base_color', '#0000ff'))}."

    higher_text = "Higher values are treated as better." if style.get("higher_is_better", True) else "Lower values are treated as better."

    return [
        "<b>Data analysis</b>",
        f"Parameter: {escape(str(metric.get('label', 'n/a')))}",
        f"Lowest value: {_format_num(value_min, 2)}",
        f"Highest value: {_format_num(value_max, 2)}",
        escape(colour_description),
        escape(higher_text),
    ]


def _data_analysis_info_html(metric: Dict[str, Any], style: Dict[str, Any], value_min: float, value_max: float) -> str:
    return _stats_lines_to_html(_data_analysis_info_lines(metric, style, value_min, value_max))


def build_heatmap_leaflet_html(
    heatmap_segments: List[List[Dict[str, Any]]],
    metric: Dict[str, Any],
    style: Dict[str, Any],
    value_min: float,
    value_max: float,
    stats_html: str,
    initial_tile_key: str = DEFAULT_TILE_KEY,
    title: str = "Flight Data Analysis Map",
) -> str:
    """Build a switchable-layer HTML map where the track is coloured by a selected CSV/computed value."""
    all_points = [[p["lat"], p["lon"]] for seg in heatmap_segments for p in seg]
    if not all_points:
        all_points = [[0.0, 0.0]]

    lats = [p[0] for p in all_points]
    lons = [p[1] for p in all_points]
    center = [sum(lats) / len(lats), sum(lons) / len(lons)]
    start_pt = all_points[0]
    end_pt = all_points[-1]

    map_id = "map_" + uuid.uuid4().hex
    div_id = map_id
    tile_js = _tile_js_layers(map_id, _ordered_tile_keys(initial_tile_key, ALL_TILE_KEYS))
    segment_lines_json = _heatmap_segment_js(heatmap_segments, style, value_min, value_max)
    stats_js = _stats_control_js(map_id, stats_html, "topright")

    return f"""<!DOCTYPE html>
<html>
<head>
  <meta http-equiv="content-type" content="text/html; charset=UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no" />
  <meta name="referrer" content="no-referrer-when-downgrade" />
  <title>{escape(title)}</title>
  <style>html, body {{width: 100%;height: 100%;margin: 0;padding: 0;}}</style>
  <style>#{div_id} {{position:absolute;top:0;bottom:0;right:0;left:0;}}</style>
  <style>
    .flight-stats-control {{
      text-align: left;
      max-width: 380px;
      white-space: normal;
      line-height: 1.35;
      padding: 3px 6px;
      overflow: visible;
    }}
    .flight-stats-control b {{
      display: inline-block;
      margin-top: 2px;
      margin-bottom: 1px;
    }}
  </style>
  <script src="https://cdn.jsdelivr.net/npm/leaflet@1.6.0/dist/leaflet.js"></script>
  <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/leaflet@1.6.0/dist/leaflet.css"/>
</head>
<body>
  <div class="folium-map" id="{div_id}"></div>
</body>
<script>
  var {map_id} = L.map("{div_id}", {{
      center: {json.dumps(center)},
      crs: L.CRS.EPSG3857,
      zoom: {DEFAULT_ZOOM},
      zoomControl: true,
      preferCanvas: false
  }});
{tile_js}
  L.control.scale({{
      position: "bottomleft",
      metric: true,
      imperial: false,
      maxWidth: 200
  }}).addTo({map_id});
{stats_js}
  var heatSegments = {segment_lines_json};
  for (var i = 0; i < heatSegments.length; i++) {{
      var s = heatSegments[i];
      L.polyline(s.coords, {{
          color: s.color,
          opacity: s.opacity,
          weight: 5
      }}).addTo({map_id});
  }}

  L.marker({json.dumps(start_pt)}).addTo({map_id}).bindPopup("Start");
  L.marker({json.dumps(end_pt)}).addTo({map_id}).bindPopup("End");

  {map_id}.fitBounds(L.latLngBounds({json.dumps(all_points)}), {{padding:[20,20]}});
</script>
</html>
"""


def output_path_for_heatmap(csv_path: str, metric: Dict[str, Any]) -> str:
    """Return output path like: flightname (analysis RSNR).html"""
    safe = str(metric.get("short") or metric.get("label") or "value")
    safe = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in safe).strip("_")
    base = os.path.splitext(csv_path)[0]
    return f"{base} (analysis {safe}).html"


def _default_analysis_stats_config(csv_path: str) -> Dict[str, Any]:
    """Menu option 3 uses the same all-stats default output style as the normal preset."""
    all_stat_groups = [STATS_GROUPS[num]["key"] for num in sorted(STATS_GROUPS.keys(), key=int)]
    stats_config = {
        "enabled": True,
        "groups": all_stat_groups,
        "position": "topright",
        "throttle_channel": "CH3(us)",
    }
    return maybe_remove_throttle_for_ardupilot(stats_config, csv_path)


def process_single_csv_data_analysis(csv_path: str) -> None:
    """CLI fallback for the v27 integrated analysis report."""
    print("\nIntegrated flight analysis creates the same polished interactive report for any supported raw or computed parameter.")
    initial_tile_key = choose_initial_tile_provider(BUILTIN_PRESET_INITIAL_TILE_KEY)
    options = _analysis_metric_options(csv_path)
    if not options:
        raise ValueError("No usable analysis parameters were found.")
    print("\nMain parameter to analyse:")
    for i, option in enumerate(options, start=1):
        print(f"{i}) {option['label']} ({option.get('unit','value')})")
    while True:
        raw = input(f"Choose 1-{len(options)} [1]: ").strip() or "1"
        try:
            chosen = int(raw)
            if 1 <= chosen <= len(options):
                metric = options[chosen - 1]
                break
        except Exception:
            pass
        print("Input not recognized.")

    defaults = analysis_defaults_for_csv(csv_path, metric)
    print(f"Automatic interpretation: {defaults['rule']} | thresholds {defaults['good_threshold']:.3f}, {defaults['bad_threshold']:.3f}")
    good_raw = input(f"First threshold [{defaults['good_threshold']:.3f}]: ").strip()
    bad_raw = input(f"Second threshold [{defaults['bad_threshold']:.3f}]: ").strip()
    good = float(good_raw) if good_raw else float(defaults['good_threshold'])
    bad = float(bad_raw) if bad_raw else float(defaults['bad_threshold'])
    cfg = {k: defaults[k] for k in ('rule','good_label','warn_label','bad_label')}
    cfg.update({'good_threshold': good, 'bad_threshold': bad})

    available = analysis_timeline_options_for_csv(csv_path)
    recommended = {"sats", "logged_speed", "coord_speed", "relative_alt"}
    print("\nSupporting timeline data:")
    for i, option in enumerate(available, start=1):
        mark = " [recommended]" if option["id"] in recommended else ""
        print(f"{i}) {option['label']} ({option['unit']}){mark}")
    raw = input("Choose numbers separated by spaces/commas (Enter = recommended, all = everything): ").strip().lower()
    if not raw:
        selected = [o['id'] for o in available if o['id'] in recommended]
    elif raw in ('all','a'):
        selected = [o['id'] for o in available]
    else:
        nums = [int(x) for x in re.findall(r"\d+", raw)]
        selected = [available[n-1]['id'] for n in nums if 1 <= n <= len(available)]

    privacy_config = choose_privacy_mode(stats_enabled=False)
    payload = build_flight_analysis_payload(csv_path, selected, privacy_config=privacy_config, primary_metric=metric, analysis_config=cfg)
    out_path = output_path_for_analysis_report(csv_path, metric)
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(build_flight_analysis_html(payload, initial_tile_key=initial_tile_key))
    summary = payload['summary']
    print("✅ Created integrated flight analysis report:")
    print(f"   HTML: {out_path}")
    print(f"   Parameter: {metric['label']} | valid rows: {summary['metric_valid']}/{summary['samples']}")
    print(f"   Flagged episodes: {summary['flagged_runs']} | duration: {_format_analysis_duration(summary.get('duration_s'))}")


def _augment_flight_stats_with_agl(csv_path: str, flight_stats: Dict[str, Any], status_callback: Optional[Any] = None) -> Tuple[Dict[str, Any], List[str]]:
    """Add AGL statistics using the shared terrain settings without changing source CSV data."""
    warnings: List[str] = []
    try:
        telemetry = _read_telemetry_records(csv_path)
        records = telemetry.get("records", [])
        terrain_values, terrain_source = _terrain_elevation_for_records(records, status_callback=status_callback)
        takeoff_msl, takeoff_source = _resolve_dashware_takeoff_msl(telemetry, terrain_values, status_callback=status_callback)
        agl_values: List[float] = []
        missing_terrain = 0
        missing_relative_altitude = 0
        for i, record in enumerate(records):
            rel = record.get("relative_alt")
            terrain = terrain_values[i] if i < len(terrain_values) else None
            if record.get("gps") is not None and terrain is None:
                missing_terrain += 1
            if record.get("gps") is not None and rel is None:
                missing_relative_altitude += 1
            if rel is None or terrain is None or takeoff_msl is None:
                continue
            agl = float(takeoff_msl) + float(rel) - float(terrain)
            # A negative AGL is physically impossible for a normal airborne track and usually
            # reflects DEM/altitude-model disagreement. Match Dashware behaviour and clamp it.
            agl_values.append(max(0.0, agl))
        out = dict(flight_stats)
        numeric = dict(out.get("numeric", {}))
        numeric["agl_alt"] = _list_stats(agl_values)
        out["numeric"] = numeric
        out["agl"] = {
            "terrain_source": terrain_source,
            "takeoff_source": takeoff_source,
            "covered": len(agl_values),
            "missing_terrain": missing_terrain,
            "missing_relative_altitude": missing_relative_altitude,
        }
        if not agl_values:
            warnings.append(f"AGL altitude could not be calculated: {terrain_source}")
        elif missing_terrain:
            warnings.append(f"AGL altitude terrain coverage was incomplete for {missing_terrain} GPS row(s).")
        return out, warnings
    except Exception as exc:
        out = dict(flight_stats)
        numeric = dict(out.get("numeric", {}))
        numeric["agl_alt"] = {}
        out["numeric"] = numeric
        warnings.append(f"AGL altitude calculation failed: {exc}")
        return out, warnings


def output_path_for_tile(csv_path: str, tile_key: str) -> str:
    """Return output path like: flightname (default).html or flightname (topo).html"""
    tile = TILE_PROVIDERS.get(tile_key, TILE_PROVIDERS[DEFAULT_TILE_KEY])
    suffix = tile.get("short", f"tile{tile_key}")
    base = os.path.splitext(csv_path)[0]
    return f"{base} ({suffix}).html"


def output_path_for_layers(csv_path: str) -> str:
    """Return output path like: flightname (layers).html"""
    base = os.path.splitext(csv_path)[0]
    return f"{base} (layers).html"


def process_csv_to_html(csv_path: str, run_options: Dict[str, Any]) -> int:
    """Process one CSV file using all chosen run options. Returns number of HTML files made."""
    stats_config_for_read = effective_stats_config_for_csv(run_options.get("stats_config", {}), csv_path)
    throttle_col_name = stats_config_for_read.get("throttle_channel", "CH3(us)")
    min_sats = int(run_options.get("min_sats", MIN_SATS) or MIN_SATS)
    flight_data = read_flight_data(csv_path, min_sats=min_sats, throttle_col_name=throttle_col_name)
    segments = flight_data["segments"]
    parse_stats = flight_data["parse_stats"]
    flight_stats = flight_data["flight_stats"]
    agl_warnings: List[str] = []
    if "agl_altitude" in stats_config_for_read.get("groups", []):
        flight_stats, agl_warnings = _augment_flight_stats_with_agl(csv_path, flight_stats, status_callback=print)
        for warning in agl_warnings:
            print(f"⚠️  {warning}")

    total_points = sum(len(seg) for seg in segments)
    if total_points == 0:
        print(f"⚠️  No usable GPS points found in: {csv_path}")
        return 0

    privacy_config = run_options.get("privacy_config", {"enabled": False, "meters": 0.0})
    map_segments, privacy_success = apply_privacy_trim(segments, privacy_config)
    if privacy_config.get("enabled") and not privacy_success:
        print("⚠️  Privacy trimming would remove too much of the track, so no HTML was created for this file.")
        return 0

    stats_config = effective_stats_config_for_csv(run_options.get("stats_config", {"enabled": False}), csv_path)
    stats_html = build_stats_html(flight_stats, stats_config, privacy_config)
    stats_position = stats_config.get("position", "topright")
    four_sat_warning = _four_sat_warning_text(flight_stats.get("parse", {}))
    if four_sat_warning and four_sat_warning not in stats_html:
        warning_html = _stats_lines_to_html(["<b>GPS quality</b>", four_sat_warning])
        stats_html = f"{stats_html}<br>{warning_html}" if stats_html else warning_html

    color_css = run_options["color_css"]
    map_mode = run_options["map_mode"]
    tile_keys = run_options["tile_keys"]
    initial_tile_key = run_options.get("initial_tile_key", DEFAULT_TILE_KEY)

    made = 0

    if map_mode == "layers":
        out_path = output_path_for_layers(csv_path)
        html = build_leaflet_html(
            map_segments,
            color_css=color_css,
            tile_keys=_ordered_tile_keys(initial_tile_key, ALL_TILE_KEYS),
            initial_tile_key=initial_tile_key,
            use_layer_control=True,
            stats_html=stats_html,
            stats_position=stats_position,
        )
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(html)
        made += 1

        print("✅ Created map:")
        print(f"   CSV:  {csv_path}")
        print(f"   HTML: {out_path}")
        print("   Basemap mode: switchable layer control (all basemaps)")
        _print_processing_stats(parse_stats, total_points, min_sats)
        return made

    for tile_key in tile_keys:
        out_path = output_path_for_tile(csv_path, tile_key)
        html = build_leaflet_html(
            map_segments,
            color_css=color_css,
            tile_key=tile_key,
            initial_tile_key=initial_tile_key,
            use_layer_control=False,
            stats_html=stats_html,
            stats_position=stats_position,
        )
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(html)
        made += 1

        print("✅ Created map:")
        print(f"   CSV:  {csv_path}")
        print(f"   HTML: {out_path}")
        print(f"   Basemap: {TILE_PROVIDERS.get(tile_key, TILE_PROVIDERS[DEFAULT_TILE_KEY])['name']}")
        _print_processing_stats(parse_stats, total_points, min_sats)

    return made


def _print_processing_stats(stats: Dict[str, int], total_points: int, min_sats: int = MIN_SATS) -> None:
    print(f"   Segments: {stats['segments']} | Points kept: {total_points} | Deduped: {stats['deduped']}")
    print(f"   Rows read: {stats['rows']} | Missing GPS breaks: {stats['missing_gps']} | Low-sats rows (<{min_sats}): {stats['low_sats']} | Bad GPS rows: {stats['bad_gps']}")
    four_kept = int(stats.get("four_sat_rows_kept", 0) or 0)
    four_total = int(stats.get("four_sat_rows_total", 0) or 0)
    if min_sats <= RELAXED_MIN_SATS and four_kept > 0:
        print(f"   WARNING: Included {four_kept} row(s) logged with 4 satellites. Track position may be less reliable and GPS altitude may freeze or jump.")
    elif min_sats > RELAXED_MIN_SATS and four_total > 0:
        print(f"   WARNING: Excluded {four_total} row(s) logged with 4 satellites. Distance and GPS-derived altitude may have gaps or reduced accuracy.")


def find_csv_files(root_folder: str) -> List[str]:
    csvs: List[str] = []
    for dirpath, _, filenames in os.walk(root_folder):
        for name in filenames:
            if name.lower().endswith(".csv"):
                csvs.append(os.path.join(dirpath, name))
    csvs.sort()
    return csvs



# ---------------------------------------------------------------------------
# v27 integrated flight-analysis report and Dashware/GPX enrichment
# ---------------------------------------------------------------------------

ANALYSIS_TIMELINE_FIELDS: List[Dict[str, str]] = [
    {"id": "sats", "label": "Satellites", "unit": "sats", "group": "Satellites"},
    {"id": "logged_speed", "label": "Logged GPS speed", "unit": "km/h", "group": "Speed"},
    {"id": "coord_speed", "label": "Speed from coordinates (2 s)", "unit": "km/h", "group": "Speed"},
    {"id": "relative_alt", "label": "Converted relative altitude", "unit": "m", "group": "Altitude"},
    {"id": "raw_alt", "label": "Original CSV altitude", "unit": "m", "group": "Altitude"},
    {"id": "altitude_msl", "label": "Altitude MSL (terrain takeoff reference)", "unit": "m", "group": "Altitude"},
    {"id": "terrain_msl", "label": "Terrain elevation", "unit": "m", "group": "Altitude"},
    {"id": "altitude_agl", "label": "Altitude AGL", "unit": "m", "group": "Altitude"},
    {"id": "vertical_speed", "label": "Derived vertical speed", "unit": "m/s", "group": "Vertical speed"},
    {"id": "logged_vspd", "label": "Logged VSpd / vario", "unit": "m/s", "group": "Vertical speed"},
    {"id": "temperature", "label": "Telemetry temperature", "unit": "°C", "group": "Temperature"},
    {"id": "rqly", "label": "Link quality (RQly)", "unit": "%", "group": "Signal"},
    {"id": "rsnr", "label": "RSNR", "unit": "dB", "group": "Signal"},
    {"id": "rssi", "label": "RSSI dBm (best receiver chain)", "unit": "dBm", "group": "Signal"},
    {"id": "tpwr", "label": "Transmit power", "unit": "mW", "group": "Power"},
    {"id": "current", "label": "Current", "unit": "A", "group": "Power"},
    {"id": "voltage", "label": "Receiver battery voltage", "unit": "V", "group": "Power"},
    {"id": "power", "label": "Electrical power", "unit": "W", "group": "Power"},
    {"id": "energy_used", "label": "Integrated energy used", "unit": "Wh", "group": "Energy"},
    {"id": "capacity", "label": "Capacity used", "unit": "mAh", "group": "Energy"},
    {"id": "distance_home", "label": "Distance from home", "unit": "m", "group": "Distance"},
    {"id": "cumulative_distance", "label": "Cumulative distance", "unit": "km", "group": "Distance"},
    {"id": "efficiency", "label": "Capacity efficiency", "unit": "mAh/km", "group": "Efficiency"},
    {"id": "energy_efficiency", "label": "Energy efficiency", "unit": "Wh/km", "group": "Efficiency"},
    {"id": "acceleration", "label": "Acceleration", "unit": "m/s²", "group": "Motion"},
    {"id": "turn_rate", "label": "Turn rate", "unit": "°/s", "group": "Motion"},
    {"id": "throttle", "label": "Throttle", "unit": "%", "group": "Control"},
]

DASHWARE_FIELDS: List[Dict[str, str]] = [
    {"id": "elapsed", "label": "Elapsed time (always first added column)", "column": "Elapsed_Time"},
    {"id": "latitude", "label": "Latitude split from the original GPS column", "column": "Latitude_deg"},
    {"id": "longitude", "label": "Longitude split from the original GPS column", "column": "Longitude_deg"},
    {"id": "heading_deg", "label": "True heading/course — automatically chooses original Hdg or centred 2-second GPS course", "column": "Heading_True_deg"},
    {"id": "heading_cardinal", "label": "Cardinal direction from the same automatically selected heading source", "column": "Heading_Cardinal"},
    {"id": "coord_speed", "label": "Coordinate-derived speed over 2 seconds", "column": "Coordinate_Speed"},
    {"id": "distance_home", "label": "Distance from first good GPS/home", "column": "Distance_From_Home"},
    {"id": "cumulative_distance", "label": "Cumulative GPS distance", "column": "Cumulative_Distance"},
    {"id": "altitude_msl", "label": "Alt MSL — terrain takeoff elevation + original relative Alt", "column": "Altitude_MSL"},
    {"id": "terrain_msl", "label": "Terrain elevation below the aircraft", "column": "Terrain_Elevation"},
    {"id": "altitude_agl", "label": "Alt AGL — aircraft height above terrain; negative terrain-model results are written as 0", "column": "Altitude_AGL"},
    {"id": "vertical_speed", "label": "Derived vertical speed from altitude", "column": "Vertical_Speed"},
    {"id": "logged_vspd", "label": "Logged vertical speed / VSpd (uses the original telemetry sensor when present)", "column": "Logged_VSpd"},
    {"id": "temperature", "label": "Telemetry temperature (when a Temp sensor is present)", "column": "Temperature_C"},
    {"id": "acceleration", "label": "Longitudinal acceleration from speed", "column": "Acceleration"},
    {"id": "turn_rate", "label": "Ground-track turn rate from the selected heading/course", "column": "Turn_Rate"},
    {"id": "roll_rate", "label": "Roll angular rate derived from the original Roll angle", "column": "Roll_Rate"},
    {"id": "pitch_rate", "label": "Pitch angular rate derived from the original pitch angle", "column": "Pitch_Rate"},
    {"id": "yaw_rate", "label": "Yaw angular rate derived from the original Yaw angle", "column": "Yaw_Rate"},
    {"id": "rssi", "label": "RSSI dBm using the best usable 1RSS/2RSS value", "column": "RSSI_Best_dBm"},
    {"id": "power", "label": "Electrical power (current × RxBt)", "column": "Power_W"},
    {"id": "energy_used", "label": "Integrated energy used", "column": "Energy_Used_Wh"},
    {"id": "efficiency_mah", "label": "Capacity efficiency", "column": "Efficiency_mAh"},
    {"id": "efficiency_wh", "label": "Energy efficiency", "column": "Efficiency_Wh"},
    {"id": "throttle", "label": "Throttle percentage (whole numbers)", "column": "Throttle_pct"},
]

DASHWARE_DEFAULT_FIELD_IDS = {
    "elapsed", "latitude", "longitude", "heading_deg", "heading_cardinal", "coord_speed",
    "distance_home", "cumulative_distance", "power", "rssi",
}

def _median(values: List[float]) -> Optional[float]:
    clean = [float(v) for v in values if isinstance(v, (int, float)) and math.isfinite(float(v))]
    if not clean:
        return None
    return float(statistics.median(clean))


def _percentile(values: List[float], percentile: float) -> Optional[float]:
    clean = sorted(float(v) for v in values if isinstance(v, (int, float)) and math.isfinite(float(v)))
    if not clean:
        return None
    if len(clean) == 1:
        return clean[0]
    p = max(0.0, min(100.0, float(percentile))) / 100.0
    position = p * (len(clean) - 1)
    lo = int(math.floor(position))
    hi = int(math.ceil(position))
    if lo == hi:
        return clean[lo]
    fraction = position - lo
    return clean[lo] * (1.0 - fraction) + clean[hi] * fraction


def _bearing_true_deg(a: Tuple[float, float], b: Tuple[float, float]) -> Optional[float]:
    if haversine_m(a, b) < 0.5:
        return None
    lat1 = math.radians(a[0])
    lat2 = math.radians(b[0])
    dlon = math.radians(b[1] - a[1])
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def _cardinal_from_heading(value: Optional[float]) -> str:
    if value is None:
        return ""
    labels = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    return labels[int((float(value) + 22.5) // 45.0) % 8]


def _angle_difference_deg(new: float, old: float) -> float:
    return ((float(new) - float(old) + 180.0) % 360.0) - 180.0


def _format_analysis_duration(seconds: Optional[float]) -> str:
    if seconds is None:
        return "n/a"
    seconds = max(0.0, float(seconds))
    if seconds < 60:
        return f"{seconds:.1f} s"
    if seconds < 3600:
        return f"{seconds / 60.0:.1f} min"
    return f"{int(seconds // 3600)} h {(seconds % 3600) / 60.0:.1f} min"


def _format_csv_number(value: Optional[float], decimals: int = 3) -> str:
    if value is None or not math.isfinite(float(value)):
        return ""
    text = f"{float(value):.{decimals}f}"
    return text.rstrip("0").rstrip(".") if "." in text else text


def _format_dashware_number(value: Optional[float], decimals: int) -> str:
    """Write exactly the number of decimals selected in the Dashware GUI."""
    if value is None or not math.isfinite(float(value)):
        return ""
    return f"{float(value):.{max(0, int(decimals))}f}"


def _read_telemetry_records(csv_path: str, throttle_col_name: str = "CH3(us)") -> Dict[str, Any]:
    """Read an EdgeTX-style CSV while preserving every original row for enrichment.

    Semantic fields are resolved by header meaning rather than position.  If EdgeTX logs
    duplicate sensor names, numeric candidates are content-scored so the populated,
    changing sensor wins over an empty/constant placeholder.  Local Date+Time remains the
    clock used for elapsed-time calculations; a newer CRSF/GPS combined UTC datetime is
    retained separately when present.
    """
    with open(csv_path, "r", encoding="utf-8-sig", errors="replace", newline="") as f:
        dialect = sniff_dialect(f)
        reader = csv.reader(f, dialect)
        header = next(reader, None)
        if not header:
            raise ValueError("The CSV is empty.")
        original_rows = [list(row) for row in reader]

    datetime_idx = _select_datetime_columns(header, original_rows)
    flight_stack = _detect_flight_stack_from_table(header, original_rows)
    fm_idx = flight_stack.get("mode_index")
    idx = {
        "date": datetime_idx.get("date"),
        "time": datetime_idx.get("time"),
        "utc_datetime": datetime_idx.get("utc_datetime"),
        "gps": _find_any_col_index(header, ["GPS", "GPS Coordinates", "GPSCoord"]),
        "sats": _best_numeric_col_index(header, original_rows, ["Sats", "Sats(#)", "Satellites"]),
        "gspd": _best_numeric_col_index(header, original_rows, ["GSpd(kmh)", "GSpd(km/h)", "GSpd", "GroundSpeed", "Ground Speed"]),
        "heading": _best_numeric_col_index(header, original_rows, ["Hdg(°)", "Hdg", "Heading(°)", "Heading"]),
        "alt": _best_numeric_col_index(header, original_rows, ["Alt(m)", "Alt", "alt (m)", "Altitude(m)", "Altitude"]),
        "vspd": _best_numeric_col_index(header, original_rows, ["VSpd(m/s)", "VSpd", "VSpeed(m/s)", "VerticalSpeed(m/s)", "Vario(m/s)"]),
        "temperature": _best_numeric_col_index(header, original_rows, ["Temp(°C)", "Temp(C)", "Temp(°)", "Temp", "Temperature(°C)", "Temperature"]),
        "pitch": _best_numeric_col_index(header, original_rows, ["Ptch(rad)", "Pitch(rad)", "Ptch(deg)", "Pitch(deg)", "Ptch(°)", "Pitch(°)", "Ptch", "Pitch"]),
        "roll": _best_numeric_col_index(header, original_rows, ["Roll(rad)", "Roll(deg)", "Roll(°)", "Roll"]),
        "yaw": _best_numeric_col_index(header, original_rows, ["Yaw(rad)", "Yaw(deg)", "Yaw(°)", "Yaw"]),
        "rqly": _best_numeric_col_index(header, original_rows, ["RQly(%)", "RQly", "LQ", "LinkQuality"]),
        "rsnr": _best_numeric_col_index(header, original_rows, ["RSNR(dB)", "RSNR(dBm)", "RSNR", "SNR"]),
        "rssi1": _best_numeric_col_index(header, original_rows, ["1RSS(dB)", "1RSS(dBm)", "1RSS", "RSSI1"]),
        "rssi2": _best_numeric_col_index(header, original_rows, ["2RSS(dB)", "2RSS(dBm)", "2RSS", "RSSI2"]),
        "tpwr": _best_numeric_col_index(header, original_rows, ["TPWR(mW)", "TPWR", "TxPower(mW)", "Tx Power"]),
        "rxbt": _best_numeric_col_index(header, original_rows, ["RxBt(V)", "RxBt", "Voltage(V)", "Volt(V)"]),
        "curr": _best_numeric_col_index(header, original_rows, ["Curr(A)", "Curr", "Current(A)", "Current"]),
        "capa": _best_numeric_col_index(header, original_rows, ["Capa(mAh)", "Capa", "Capacity(mAh)", "Capacity"]),
        "flight_mode": int(fm_idx) if isinstance(fm_idx, int) else _find_any_col_index(header, ["FM", "FlightMode", "Flight Mode", "Mode"]),
        "throttle": _best_numeric_col_index(header, original_rows, [throttle_col_name, throttle_col_name.replace("(us)", ""), "CH3(us)", "CH3"]),
    }
    if idx["gps"] is None:
        raise ValueError("No GPS column was found.")

    filename_dt = _filename_local_datetime(csv_path)
    filename_date_text = filename_dt.strftime("%Y-%m-%d") if filename_dt is not None else ""
    utc_candidates: List[Optional[datetime]] = []
    if idx.get("utc_datetime") is not None:
        utc_candidates = [_parse_combined_datetime_text(_clean_cell(row, idx.get("utc_datetime"))) for row in original_rows]
    else:
        utc_candidates = [None] * len(original_rows)
    first_utc_dt = next((dt for dt in utc_candidates if isinstance(dt, datetime)), None)
    records: List[Dict[str, Any]] = []
    parsed_times: List[Optional[float]] = []
    for row_index, row in enumerate(original_rows):
        date_text = _clean_cell(row, idx["date"])
        time_text = _clean_cell(row, idx["time"])
        utc_dt = utc_candidates[row_index] if row_index < len(utc_candidates) else None
        # If a future log loses the separate local Date but retains local Time, the filename
        # is a safe date anchor because EdgeTX names the log from the local start clock.
        effective_date = date_text or (filename_date_text if time_text else "")
        effective_time = time_text
        # Stronger fallback: if local clock fields disappear but the new GPS UTC sensor and
        # EdgeTX filename timestamp exist, preserve the filename's local start and advance it
        # by the GPS-UTC elapsed delta. This avoids guessing a political timezone from longitude.
        if not effective_time and filename_dt is not None and isinstance(utc_dt, datetime) and isinstance(first_utc_dt, datetime):
            try:
                derived_local = filename_dt + (utc_dt.replace(tzinfo=None) - first_utc_dt.replace(tzinfo=None))
                effective_date = derived_local.strftime("%Y-%m-%d")
                effective_time = derived_local.strftime("%H:%M:%S.%f").rstrip("0").rstrip(".")
            except Exception:
                pass
        parsed_time = _parse_datetime_value(effective_date, effective_time)
        parsed_times.append(parsed_time)
        gps = _parse_gps_cell(_clean_cell(row, idx["gps"]))
        sats = _parse_sats(_clean_cell(row, idx["sats"])) if idx["sats"] is not None else None
        rssi1 = _parse_float(_clean_cell(row, idx["rssi1"]))
        rssi2 = _parse_float(_clean_cell(row, idx["rssi2"]))
        throttle_us = _parse_float(_clean_cell(row, idx["throttle"]))
        throttle_pct = None
        if throttle_us is not None:
            throttle_pct = max(0.0, min(100.0, (throttle_us - THROTTLE_MIN_US) / (THROTTLE_MAX_US - THROTTLE_MIN_US) * 100.0))
        heading_original = _parse_float(_clean_cell(row, idx["heading"]))
        utc_text = _clean_cell(row, idx["utc_datetime"])
        mode_raw = _clean_cell(row, idx["flight_mode"])
        mode = _normalise_flight_mode_token(mode_raw)
        record = {
            "row_index": row_index,
            "row": row,
            "date": date_text or effective_date,
            "time": time_text or effective_time,
            "display_time": _format_time_for_display(date_text or effective_date, time_text or effective_time),
            "time_abs": parsed_time,
            "utc_datetime_text": utc_text,
            "utc_datetime": utc_dt,
            "elapsed_s": None,
            "gps": gps,
            "lat": gps[0] if gps else None,
            "lon": gps[1] if gps else None,
            "sats": sats,
            "gspd": _parse_float(_clean_cell(row, idx["gspd"])),
            "heading_raw_original": heading_original,
            "heading_raw": _normalise_logged_heading_deg(heading_original),
            "alt_raw": _parse_float(_clean_cell(row, idx["alt"])),
            "vspd_logged_mps": _parse_float(_clean_cell(row, idx["vspd"])),
            "temperature_c": _parse_float(_clean_cell(row, idx["temperature"])),
            "pitch_raw": _parse_float(_clean_cell(row, idx["pitch"])),
            "roll_raw": _parse_float(_clean_cell(row, idx["roll"])),
            "yaw_raw": _parse_float(_clean_cell(row, idx["yaw"])),
            "rqly": _parse_float(_clean_cell(row, idx["rqly"])),
            "rsnr": _parse_float(_clean_cell(row, idx["rsnr"])),
            "rssi1": rssi1,
            "rssi2": rssi2,
            "rssi": _best_rssi_from_values(rssi1, rssi2),
            "tpwr": _parse_float(_clean_cell(row, idx["tpwr"])),
            "rxbt": _parse_float(_clean_cell(row, idx["rxbt"])),
            "curr": _parse_float(_clean_cell(row, idx["curr"])),
            "capa": _parse_float(_clean_cell(row, idx["capa"])),
            "flight_mode_raw": mode_raw,
            "flight_mode": mode,
            "throttle_us": throttle_us,
            "throttle_pct": throttle_pct,
        }
        record["power_w"] = record["curr"] * record["rxbt"] if record["curr"] is not None and record["rxbt"] is not None else None
        records.append(record)

    # Build an aligned, monotonic elapsed-time series from the local EdgeTX clock. Missing
    # timestamps use the median observed row interval; the separate GPS UTC field never
    # replaces the user's local Date/Time for app timing/statistics.
    valid_deltas: List[float] = []
    previous_valid: Optional[float] = None
    rollover_offset = 0.0
    adjusted: List[Optional[float]] = []
    for value in parsed_times:
        if value is None:
            adjusted.append(None)
            continue
        candidate = float(value) + rollover_offset
        if previous_valid is not None and candidate < previous_valid - 12 * 3600:
            rollover_offset += 86400.0
            candidate = float(value) + rollover_offset
        if previous_valid is not None and candidate > previous_valid:
            valid_deltas.append(candidate - previous_valid)
        previous_valid = candidate
        adjusted.append(candidate)
    sample_period = _median([d for d in valid_deltas if 0 < d < 60]) or 0.2
    first_valid = next((v for v in adjusted if v is not None), 0.0)
    last_elapsed = 0.0
    for i, value in enumerate(adjusted):
        if value is not None:
            elapsed = max(last_elapsed, float(value) - float(first_valid))
        else:
            elapsed = last_elapsed + sample_period if i > 0 else 0.0
        records[i]["elapsed_s"] = elapsed
        last_elapsed = elapsed

    autonomy = _assess_autonomy_from_records(records, flight_stack, sample_period)
    altitude = _normalise_altitude_records(records, str(flight_stack.get("stack", "unknown")))
    for record, rel, msl in zip(records, altitude["relative"], altitude["msl"]):
        record["relative_alt"] = rel
        record["altitude_msl"] = msl

    _compute_gps_derived_fields(records)
    attitude_headers = {
        axis: (header[idx[axis]] if idx.get(axis) is not None and idx[axis] < len(header) else "")
        for axis in ("roll", "pitch", "yaw")
    }
    _compute_attitude_fields(records, attitude_headers)

    observed_utc_minus_local_hours: Optional[float] = None
    for record in records:
        utc_dt = record.get("utc_datetime")
        if not isinstance(utc_dt, datetime) or not record.get("date") or not record.get("time"):
            continue
        try:
            local_dt = datetime.fromisoformat(f"{record['date']} {record['time']}")
            delta_h = (utc_dt.replace(tzinfo=None) - local_dt.replace(tzinfo=None)).total_seconds() / 3600.0
            while delta_h > 14.0:
                delta_h -= 24.0
            while delta_h < -14.0:
                delta_h += 24.0
            observed_utc_minus_local_hours = delta_h
            break
        except Exception:
            pass

    return {
        "header": header,
        "rows": original_rows,
        "dialect": dialect,
        "indices": idx,
        "records": records,
        "sample_period": sample_period,
        "altitude_source": altitude["source"],
        "csv_takeoff_msl": altitude["takeoff_msl"],
        "attitude_headers": attitude_headers,
        "flight_stack": flight_stack,
        "autonomy": autonomy,
        "datetime_sources": {
            "local_date_index": idx.get("date"),
            "local_time_index": idx.get("time"),
            "utc_datetime_index": idx.get("utc_datetime"),
            "filename_fallback_used": bool(filename_date_text and idx.get("date") is None),
            "observed_utc_minus_local_hours": observed_utc_minus_local_hours,
            "strategy": "local EdgeTX Date+Time; GPS UTC retained separately; filename+UTC-delta fallback if local clock fields are absent",
        },
    }

def _normalise_altitude_records(records: List[Dict[str, Any]], flight_stack: str = "unknown") -> Dict[str, Any]:
    """Normalise ArduPilot ASL and Betaflight relative-altitude logs without creating artificial gaps.

    Betaflight logs may contain one or several takeoff-elevation samples and then reset to
    zero. Once that reset is seen, every later value is treated as relative altitude unless
    a sudden isolated value clearly looks like an MSL reversion. A normal climb through
    50 m, 500 m, or higher must never be mistaken for an MSL sample.
    """
    values = [float(r["alt_raw"]) for r in records if r.get("alt_raw") is not None]
    source = detect_altitude_source_from_values(values, flight_stack)
    relative: List[Optional[float]] = [None] * len(records)
    msl: List[Optional[float]] = [None] * len(records)
    takeoff_msl: Optional[float] = None

    if not values:
        return {"source": "missing", "takeoff_msl": None, "relative": relative, "msl": msl}

    valid_pairs = [(i, float(r["alt_raw"])) for i, r in enumerate(records) if r.get("alt_raw") is not None]
    if _altitude_source_is_asl(source):
        first_values = [v for _i, v in valid_pairs[: min(30, len(valid_pairs))]]
        takeoff_msl = _median(first_values)
        for i, raw in valid_pairs:
            msl[i] = raw
            relative[i] = raw - takeoff_msl if takeoff_msl is not None else None
        return {"source": source, "takeoff_msl": takeoff_msl, "relative": relative, "msl": msl}

    if source in ("ardupilot_relative", "inav_relative", "relative_unknown"):
        # These sources already report relative altitude from/near home.  Do not apply the
        # Betaflight initial-MSL reset heuristic merely because the log begins at zero.
        for i, raw in valid_pairs:
            relative[i] = raw
        return {"source": source, "takeoff_msl": None, "relative": relative, "msl": msl}

    reset_index: Optional[int] = None
    high_before: List[float] = []
    for pair_pos, (row_i, raw) in enumerate(valid_pairs[:300]):
        if abs(raw) >= ALT_MSL_LOOKING_THRESHOLD_M:
            high_before.append(raw)
            for later_i, later_raw in valid_pairs[pair_pos + 1:pair_pos + 181]:
                if abs(later_raw) <= ALT_RELATIVE_ZERO_THRESHOLD_M:
                    reset_index = later_i
                    break
        if reset_index is not None:
            break
    if reset_index is not None and high_before:
        takeoff_msl = _median(high_before)

    previous_rel: Optional[float] = None
    for i, raw in valid_pairs:
        if reset_index is not None and i < reset_index:
            # Preserve the pre-reset MSL-looking value only as metadata; it is not flight-relative altitude.
            msl[i] = raw
            continue

        candidate = raw
        if previous_rel is not None and abs(candidate - previous_rel) > ALT_RELATIVE_SPIKE_JUMP_THRESHOLD_M:
            # A genuine temporary return to MSL usually becomes plausible again after subtracting
            # the known takeoff elevation. Do that only for a sudden jump, never merely because
            # the value is greater than 50 m.
            possible_relative = raw - takeoff_msl if takeoff_msl is not None else None
            if possible_relative is not None and abs(possible_relative - previous_rel) <= ALT_RELATIVE_SPIKE_JUMP_THRESHOLD_M:
                candidate = possible_relative
            else:
                continue

        relative[i] = candidate
        previous_rel = candidate
        if takeoff_msl is not None:
            msl[i] = takeoff_msl + candidate

    return {"source": source, "takeoff_msl": takeoff_msl, "relative": relative, "msl": msl}

def _compute_gps_derived_fields(records: List[Dict[str, Any]]) -> None:
    valid_indices = [i for i, r in enumerate(records) if r.get("gps") is not None]
    if not valid_indices:
        return
    home = records[valid_indices[0]]["gps"]
    cumulative_m = 0.0
    previous_valid: Optional[int] = None
    start_capa = next((float(r["capa"]) for r in records if r.get("capa") is not None), None)
    energy_wh = 0.0

    for i, record in enumerate(records):
        gps = record.get("gps")
        if gps is not None:
            if previous_valid is not None:
                cumulative_m += haversine_m(records[previous_valid]["gps"], gps)
            previous_valid = i
            record["distance_home_m"] = haversine_m(home, gps)
        else:
            record["distance_home_m"] = None
        record["cumulative_distance_km"] = cumulative_m / 1000.0
        capa = record.get("capa")
        record["capacity_used_mAh"] = max(0.0, float(capa) - start_capa) if capa is not None and start_capa is not None else None
        if i > 0:
            dt = max(0.0, float(record["elapsed_s"]) - float(records[i - 1]["elapsed_s"]))
            p0 = records[i - 1].get("power_w")
            p1 = record.get("power_w")
            if p0 is not None and p1 is not None and dt <= 60:
                energy_wh += ((float(p0) + float(p1)) / 2.0) * dt / 3600.0
        record["energy_used_Wh"] = energy_wh
        if cumulative_m >= 100.0:
            record["efficiency_mAh_km"] = record["capacity_used_mAh"] / (cumulative_m / 1000.0) if record.get("capacity_used_mAh") is not None else None
            record["efficiency_Wh_km"] = energy_wh / (cumulative_m / 1000.0)
        else:
            record["efficiency_mAh_km"] = None
            record["efficiency_Wh_km"] = None

    # Coordinate speed and true course use an approximately two-second centred window.
    for pos, i in enumerate(valid_indices):
        t = float(records[i]["elapsed_s"])
        left_pos = pos
        while left_pos > 0 and t - float(records[valid_indices[left_pos]]["elapsed_s"]) < ANALYSIS_COORD_SPEED_WINDOW_S / 2.0:
            left_pos -= 1
        right_pos = pos
        while right_pos + 1 < len(valid_indices) and float(records[valid_indices[right_pos]]["elapsed_s"]) - t < ANALYSIS_COORD_SPEED_WINDOW_S / 2.0:
            right_pos += 1
        a_i = valid_indices[left_pos]
        b_i = valid_indices[right_pos]
        dt = float(records[b_i]["elapsed_s"]) - float(records[a_i]["elapsed_s"])
        if b_i != a_i and dt > 0:
            distance = haversine_m(records[a_i]["gps"], records[b_i]["gps"])
            records[i]["coord_speed_kmh"] = distance / dt * 3.6
            heading = _bearing_true_deg(records[a_i]["gps"], records[b_i]["gps"])
            if records[i]["coord_speed_kmh"] >= 1.0:
                records[i]["heading_true_deg"] = heading
                records[i]["heading_cardinal"] = _cardinal_from_heading(heading)
            else:
                records[i]["heading_true_deg"] = None
                records[i]["heading_cardinal"] = ""
        else:
            records[i]["coord_speed_kmh"] = None
            records[i]["heading_true_deg"] = None
            records[i]["heading_cardinal"] = ""

    def derivative(field: str, output: str, scale: float = 1.0, angular: bool = False) -> None:
        previous: Optional[int] = None
        for i, record in enumerate(records):
            value = record.get(field)
            if value is None:
                record[output] = None
                continue
            if previous is None:
                record[output] = None
                previous = i
                continue
            dt = float(record["elapsed_s"]) - float(records[previous]["elapsed_s"])
            if dt <= 0 or dt > 10:
                record[output] = None
            else:
                delta = _angle_difference_deg(float(value), float(records[previous][field])) if angular else float(value) - float(records[previous][field])
                record[output] = delta * scale / dt
            previous = i

    derivative("relative_alt", "vertical_speed_mps")
    derivative("gspd", "acceleration_mps2", scale=1.0 / 3.6)
    derivative("heading_true_deg", "turn_rate_deg_s", angular=True)


def analysis_timeline_options_for_csv(csv_path: str) -> List[Dict[str, str]]:
    """Return computed timeline choices plus every useful numeric original CSV column."""
    data = _read_telemetry_records(csv_path)
    records = data["records"]
    available: List[Dict[str, str]] = []
    field_map = {
        "sats": "sats", "logged_speed": "gspd", "coord_speed": "coord_speed_kmh",
        "relative_alt": "relative_alt", "raw_alt": "alt_raw", "vertical_speed": "vertical_speed_mps",
        "logged_vspd": "vspd_logged_mps", "temperature": "temperature_c",
        "rqly": "rqly", "rsnr": "rsnr", "rssi": "rssi", "tpwr": "tpwr",
        "current": "curr", "voltage": "rxbt", "power": "power_w", "energy_used": "energy_used_Wh",
        "capacity": "capacity_used_mAh", "distance_home": "distance_home_m",
        "cumulative_distance": "cumulative_distance_km", "efficiency": "efficiency_mAh_km",
        "energy_efficiency": "efficiency_Wh_km", "acceleration": "acceleration_mps2",
        "turn_rate": "turn_rate_deg_s", "throttle": "throttle_pct",
    }
    has_gps = any(r.get("gps") is not None for r in records)
    has_alt = any(r.get("relative_alt") is not None for r in records)
    seen_labels: set[str] = set()
    for option in ANALYSIS_TIMELINE_FIELDS:
        field_id = option["id"]
        include = False
        if field_id in ("altitude_msl", "terrain_msl", "altitude_agl"):
            include = has_gps and has_alt
        else:
            field = field_map.get(field_id)
            include = bool(field and any(r.get(field) is not None for r in records))
        if include:
            item = dict(option)
            available.append(item)
            seen_labels.add(_analysis_col_key(str(item.get("label", ""))))

    # The main analysis menu already determines which original columns are numeric.
    # Expose those same original samples as optional comparison traces without altering them.
    for metric in _analysis_metric_options(csv_path):
        if metric.get("type") != "raw":
            continue
        column = str(metric.get("column") or "").strip()
        if not column:
            continue
        label = str(metric.get("label") or column)
        label_key = _analysis_col_key(label)
        if label_key in seen_labels:
            continue
        unit = str(metric.get("unit") or "value")
        available.append({
            "id": f"raw::{column}",
            "label": f"Original CSV: {label}",
            "unit": unit,
            "group": f"Original CSV — {unit}",
            "type": "raw",
            "column": column,
            "column_index": metric.get("column_index"),
        })
        seen_labels.add(label_key)
    return available

def _trim_analysis_records_for_privacy(records: List[Dict[str, Any]], privacy_config: Dict[str, Any]) -> List[Dict[str, Any]]:
    copied = [dict(r) for r in records]
    if not privacy_config.get("enabled"):
        return copied
    valid = [i for i, r in enumerate(copied) if r.get("gps") is not None and (r.get("sats") is None or float(r.get("sats")) >= ANALYSIS_MIN_TRACK_SATS)]
    if len(valid) < 2:
        return copied
    cumulative = [0.0]
    for a, b in zip(valid, valid[1:]):
        cumulative.append(cumulative[-1] + haversine_m(copied[a]["gps"], copied[b]["gps"]))
    total = cumulative[-1]
    start_m = float(privacy_config.get("start_meters", privacy_config.get("meters", 0.0)) or 0.0)
    end_m = float(privacy_config.get("end_meters", privacy_config.get("meters", 0.0)) or 0.0)
    for pos, i in enumerate(valid):
        if cumulative[pos] < start_m or (total - cumulative[pos]) < end_m:
            copied[i]["gps"] = None
            copied[i]["lat"] = None
            copied[i]["lon"] = None
    return copied


def _analysis_metric_value_map(records: List[Dict[str, Any]], data: Dict[str, Any], option: Dict[str, Any], status_callback: Optional[Any] = None, allow_terrain: bool = True, terrain_context: Optional[Dict[str, Any]] = None) -> Tuple[List[Optional[float]], str]:
    """Return one value per record for a raw or computed analysis parameter."""
    metric_id = str(option.get("id", ""))
    if option.get("type") == "raw":
        try:
            idx = int(option.get("column_index")) if option.get("column_index") is not None else None
        except Exception:
            idx = None
        if idx is None or not (0 <= idx < len(data["header"])):
            idx = _best_numeric_col_index(data["header"], [r["row"] for r in records], [str(option.get("column", ""))])
        return ([_parse_float(_clean_cell(r["row"], idx)) for r in records], "Original CSV column")

    mapping = {
        "satellites": "sats", "rssi_best": "rssi", "coordinate_speed_kmh": "coord_speed_kmh",
        "power_w": "power_w", "energy_used_Wh": "energy_used_Wh",
        "efficiency_mAh_km": "efficiency_mAh_km", "energy_rate_Wh_km": "efficiency_Wh_km",
        "distance_home_m": "distance_home_m", "cumulative_distance_km": "cumulative_distance_km",
        "relative_alt_m": "relative_alt", "climb_rate_ms": "vertical_speed_mps",
        "acceleration_mps2": "acceleration_mps2", "turn_rate_deg_s": "turn_rate_deg_s",
        "throttle_pct_ch3": "throttle_pct", "capacity_used_mAh": "capacity_used_mAh",
    }
    field = mapping.get(metric_id)
    if field:
        return ([r.get(field) for r in records], "Computed from the CSV without changing original samples")

    if metric_id in ("altitude_msl", "terrain_msl", "altitude_agl"):
        if not allow_terrain:
            return ([None] * len(records), "Terrain lookup occurs when the report is generated")
        context = terrain_context if terrain_context is not None else {}
        terrain = context.get("terrain")
        terrain_source = context.get("terrain_source")
        takeoff = context.get("takeoff")
        takeoff_source = context.get("takeoff_source")
        if terrain is None:
            terrain, terrain_source = _terrain_elevation_for_records(records, status_callback=status_callback)
            context["terrain"] = terrain
            context["terrain_source"] = terrain_source
        if "takeoff" not in context:
            takeoff, takeoff_source = _resolve_dashware_takeoff_msl(data, terrain, status_callback=status_callback)
            context["takeoff"] = takeoff
            context["takeoff_source"] = takeoff_source
        values: List[Optional[float]] = []
        for record, ground in zip(records, terrain):
            rel = record.get("relative_alt")
            aircraft_msl = takeoff + float(rel) if takeoff is not None and rel is not None else None
            if metric_id == "terrain_msl": values.append(ground)
            elif metric_id == "altitude_msl": values.append(aircraft_msl)
            else: values.append(aircraft_msl - ground if aircraft_msl is not None and ground is not None else None)
        return values, f"{terrain_source}; takeoff reference: {takeoff_source}"
    return ([None] * len(records), "No value source")


def _analysis_default_rule(option: Dict[str, Any], values: List[Optional[float]]) -> Dict[str, Any]:
    clean = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    q25 = _percentile(clean, 25) if clean else 0.0
    q75 = _percentile(clean, 75) if clean else 1.0
    metric_id = str(option.get("id", ""))
    key = (str(option.get("label", "")) + " " + metric_id).lower()
    if metric_id == "satellites" or "sats" in key or "satellite" in key:
        return {"rule": "higher", "good_threshold": 5.0, "bad_threshold": 4.0, "good_label": "Trusted (5+)", "warn_label": "Marginal (4)", "bad_label": "Below 4 / missing"}
    if "rqly" in key or "link quality" in key:
        return {"rule": "higher", "good_threshold": 95.0, "bad_threshold": 70.0, "good_label": "Good", "warn_label": "Caution", "bad_label": "Poor"}
    if "rsnr" in key:
        return {"rule": "higher", "good_threshold": 5.0, "bad_threshold": 0.0, "good_label": "Good", "warn_label": "Caution", "bad_label": "Poor"}
    if "rssi" in key:
        return {"rule": "higher", "good_threshold": -70.0, "bad_threshold": -95.0, "good_label": "Good", "warn_label": "Caution", "bad_label": "Poor"}
    profile = str(option.get("profile", "bands"))
    if profile == "higher":
        return {"rule": "higher", "good_threshold": float(q75 or 0.0), "bad_threshold": float(q25 or 0.0), "good_label": "High / good", "warn_label": "Middle", "bad_label": "Low / poor"}
    if profile == "lower":
        return {"rule": "lower", "good_threshold": float(q25 or 0.0), "bad_threshold": float(q75 or 0.0), "good_label": "Low / good", "warn_label": "Middle", "bad_label": "High / poor"}
    return {"rule": "bands", "good_threshold": float(q25 or 0.0), "bad_threshold": float(q75 or 0.0), "good_label": "Low", "warn_label": "Medium", "bad_label": "High"}


def analysis_defaults_for_csv(csv_path: str, option: Dict[str, Any]) -> Dict[str, Any]:
    data = _read_telemetry_records(csv_path)
    values, note = _analysis_metric_value_map(data["records"], data, option, allow_terrain=False)
    result = _analysis_default_rule(option, values)
    clean = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    result.update({"min": min(clean) if clean else None, "median": _median(clean), "max": max(clean) if clean else None, "source_note": note})
    return result


def _analysis_classify(value: Optional[float], config: Dict[str, Any]) -> str:
    if value is None or not math.isfinite(float(value)):
        return "missing"
    v = float(value); rule = str(config.get("rule", "bands"))
    good = float(config.get("good_threshold", 0.0)); bad = float(config.get("bad_threshold", 0.0))
    if rule == "higher":
        if v >= good: return "good"
        if v < bad: return "bad"
        return "warn"
    if rule == "lower":
        if v <= good: return "good"
        if v > bad: return "bad"
        return "warn"
    low, high = sorted((good, bad))
    if v < low: return "good"
    if v >= high: return "bad"
    return "warn"


def _analysis_episode_groups(records: List[Dict[str, Any]], sample_period: float, unit: str) -> List[Dict[str, Any]]:
    episodes: List[Dict[str, Any]] = []
    current: List[Dict[str, Any]] = []
    current_class = ""
    def close() -> None:
        nonlocal current, current_class
        if not current: return
        vals = [float(r["analysis_value"]) for r in current if r.get("analysis_value") is not None]
        coords = [[float(r["lat"]), float(r["lon"])] for r in current if r.get("gps") is not None]
        duration = max(sample_period, float(current[-1]["elapsed_s"]) - float(current[0]["elapsed_s"]) + sample_period)
        episodes.append({
            "class": current_class, "start_time": current[0].get("display_time", ""), "end_time": current[-1].get("display_time", ""),
            "start_x": current[0].get("plot_time"), "end_x": current[-1].get("plot_time"), "duration_s": duration,
            "min_value": min(vals) if vals else None, "avg_value": sum(vals)/len(vals) if vals else None, "max_value": max(vals) if vals else None,
            "coords": coords, "finding": f"{current_class.title()} {unit} interval",
        })
        current=[]; current_class=""
    previous_t: Optional[float] = None
    for record in records:
        cls = str(record.get("analysis_class", "missing"))
        t = float(record.get("elapsed_s") or 0.0)
        flagged = cls in ("warn", "bad") and record.get("analysis_value") is not None
        gap = previous_t is not None and t - previous_t > max(3.0, sample_period * 4.0)
        if not flagged or gap or (current and cls != current_class): close()
        if flagged:
            if not current: current_class = cls
            current.append(record)
        previous_t = t
    close()
    return episodes


def _analysis_route_by_class(records: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    segments: List[Dict[str, Any]] = []
    current: List[List[float]] = []
    current_class = ""
    def close() -> None:
        nonlocal current, current_class
        if current:
            segments.append({"class": current_class or "missing", "coords": current})
        current=[]; current_class=""
    for record in records:
        gps_ok = record.get("gps") is not None and (record.get("sats") is None or float(record.get("sats")) >= ANALYSIS_MIN_TRACK_SATS)
        if not gps_ok:
            close(); continue
        cls = str(record.get("analysis_class", "missing"))
        point = [round(float(record["lat"]), 7), round(float(record["lon"]), 7)]
        if current and cls != current_class:
            boundary = current[-1]
            close()
            current_class = cls
            current = [boundary]
        if not current:
            current_class = cls
        if not current or current[-1] != point:
            current.append(point)
    close(); return segments


def _analysis_plot_time(record: Dict[str, Any]) -> str:
    date_text = str(record.get("date") or "").strip(); time_text = str(record.get("time") or "").strip()
    if date_text and time_text:
        return f"{date_text}T{time_text}"
    base = datetime(1970, 1, 1) + timedelta(seconds=float(record.get("elapsed_s") or 0.0))
    return base.isoformat(timespec="milliseconds")


def build_flight_analysis_payload(csv_path: str, timeline_ids: List[str], privacy_config: Optional[Dict[str, Any]] = None, primary_metric: Optional[Dict[str, Any]] = None, analysis_config: Optional[Dict[str, Any]] = None, status_callback: Optional[Any] = None) -> Dict[str, Any]:
    data = _read_telemetry_records(csv_path)
    records = _trim_analysis_records_for_privacy(data["records"], privacy_config or {"enabled": False})
    if not any(r.get("gps") is not None for r in records):
        raise ValueError("No usable GPS positions remained for the analysis map.")
    options = _analysis_metric_options(csv_path)
    option = primary_metric or next((o for o in options if o.get("id") == "satellites"), options[0] if options else None)
    if option is None: raise ValueError("No analysable numeric parameter was found.")
    terrain_context: Dict[str, Any] = {}
    values, source_note = _analysis_metric_value_map(records, data, option, status_callback=status_callback, allow_terrain=True, terrain_context=terrain_context)
    defaults = _analysis_default_rule(option, values)
    config = dict(defaults); config.update(analysis_config or {})
    config["good_threshold"] = float(config.get("good_threshold", defaults["good_threshold"]))
    config["bad_threshold"] = float(config.get("bad_threshold", defaults["bad_threshold"]))
    config.setdefault("good_label", defaults["good_label"]); config.setdefault("warn_label", defaults["warn_label"]); config.setdefault("bad_label", defaults["bad_label"])

    for record, value in zip(records, values):
        record["analysis_value"] = value
        record["analysis_class"] = _analysis_classify(value, config)
        record["plot_time"] = _analysis_plot_time(record)

    route_segments = _analysis_route_by_class(records)
    distance_m = sum(haversine_m(a,b) for seg in route_segments for a,b in zip(seg["coords"], seg["coords"][1:]))
    duration_s = float(records[-1]["elapsed_s"]) - float(records[0]["elapsed_s"]) if len(records)>1 else 0.0
    clean = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    sample_period = float(data.get("sample_period") or 0.2)
    class_seconds={"good":0.0,"warn":0.0,"bad":0.0,"missing":0.0}
    for i, record in enumerate(records):
        dt = sample_period if i+1>=len(records) else max(0.0,min(5.0,float(records[i+1]["elapsed_s"])-float(record["elapsed_s"])))
        class_seconds[str(record.get("analysis_class","missing"))] += dt
    total_class = sum(class_seconds.values()) or 1.0
    episodes = _analysis_episode_groups(records,sample_period,str(option.get("unit","value")))
    all_speed=[float(r["gspd"]) for r in records if r.get("gspd") is not None]
    all_alt=[float(r["relative_alt"]) for r in records if r.get("relative_alt") is not None]
    summary={
        "samples":len(records),"sample_period":sample_period,"duration_s":duration_s,"distance_km":distance_m/1000.0,
        "max_speed":max(all_speed) if all_speed else None,"max_alt":max(all_alt) if all_alt else None,
        "metric_min":min(clean) if clean else None,"metric_avg":sum(clean)/len(clean) if clean else None,"metric_max":max(clean) if clean else None,
        "metric_valid":len(clean),"good_seconds":class_seconds["good"],"warn_seconds":class_seconds["warn"],"bad_seconds":class_seconds["bad"],
        "missing_seconds":class_seconds["missing"],"good_pct":class_seconds["good"]/total_class*100.0,"warn_pct":class_seconds["warn"]/total_class*100.0,
        "bad_pct":class_seconds["bad"]/total_class*100.0,"flagged_runs":len(episodes),
    }
    coverage=len(clean)/len(records)*100.0 if records else 0.0
    bad_pct=summary["bad_pct"]; warn_pct=summary["warn_pct"]
    badge="Mostly good" if bad_pct<1 and warn_pct<10 else "Review" if bad_pct<10 else "Attention"
    worst=max(episodes,key=lambda e:(e["class"]=="bad",e["duration_s"]),default=None)
    verdicts=[
        {"name":"Coverage","badge":"Mostly OK" if coverage>=90 else "Limited evidence","text":f"{len(clean)} of {len(records)} rows ({coverage:.1f}%) contained a usable {option['label']} value."},
        {"name":"Result","badge":"Mostly OK" if badge=="Mostly good" else "Possible issue","text":f"{summary['good_pct']:.1f}% was {config['good_label']}; {summary['warn_pct']:.1f}% was {config['warn_label']}; {summary['bad_pct']:.1f}% was {config['bad_label']}."},
        {"name":"Longest","badge":"Limited evidence" if worst is None else "Possible issue","text":"No warning/bad episode was found." if worst is None else f"Longest flagged episode lasted {worst['duration_s']:.1f} s ({worst['class']})."},
        {"name":"Source","badge":"Mostly OK","text":source_note},
    ]

    popup_points=[]; next_regular=0.0
    for r in records:
        if r.get("gps") is None: continue
        flagged=r.get("analysis_class") in ("warn","bad","missing")
        if flagged or float(r["elapsed_s"])>=next_regular:
            popup_points.append({"lat":round(float(r["lat"]),7),"lon":round(float(r["lon"]),7),"time":r.get("display_time",""),
                "value":r.get("analysis_value"),"class":r.get("analysis_class"),"sats":r.get("sats"),"speed":r.get("gspd"),
                "coord_speed":r.get("coord_speed_kmh"),"alt":r.get("relative_alt"),"rqly":r.get("rqly"),"rsnr":r.get("rsnr"),"rssi":r.get("rssi"),"power":r.get("power_w")})
            if not flagged: next_regular=float(r["elapsed_s"])+ANALYSIS_INSPECTION_INTERVAL_S

    timeline_field_map={
        "sats":"sats","logged_speed":"gspd","coord_speed":"coord_speed_kmh","relative_alt":"relative_alt","raw_alt":"alt_raw",
        "vertical_speed":"vertical_speed_mps","logged_vspd":"vspd_logged_mps","temperature":"temperature_c",
        "rqly":"rqly","rsnr":"rsnr","rssi":"rssi","tpwr":"tpwr","current":"curr","voltage":"rxbt",
        "power":"power_w","energy_used":"energy_used_Wh","capacity":"capacity_used_mAh","distance_home":"distance_home_m",
        "cumulative_distance":"cumulative_distance_km","efficiency":"efficiency_mAh_km","energy_efficiency":"efficiency_Wh_km",
        "acceleration":"acceleration_mps2","turn_rate":"turn_rate_deg_s","throttle":"throttle_pct",
    }
    terrain_cache: Dict[str,List[Optional[float]]] = {}
    requested_terrain=[x for x in timeline_ids if x in ("altitude_msl","terrain_msl","altitude_agl")]
    if requested_terrain:
        terrain = terrain_context.get("terrain")
        takeoff = terrain_context.get("takeoff")
        if terrain is None:
            terrain, terrain_source = _terrain_elevation_for_records(records,status_callback=status_callback)
            terrain_context["terrain"] = terrain
            terrain_context["terrain_source"] = terrain_source
        if "takeoff" not in terrain_context:
            takeoff, takeoff_source = _resolve_dashware_takeoff_msl(data,terrain,status_callback=status_callback)
            terrain_context["takeoff"] = takeoff
            terrain_context["takeoff_source"] = takeoff_source
        terrain_cache["terrain_msl"]=terrain
        terrain_cache["altitude_msl"]=[takeoff+float(r["relative_alt"]) if takeoff is not None and r.get("relative_alt") is not None else None for r in records]
        terrain_cache["altitude_agl"]=[a-g if a is not None and g is not None else None for a,g in zip(terrain_cache["altitude_msl"],terrain)]
    option_lookup={o["id"]:o for o in analysis_timeline_options_for_csv(csv_path)}
    series=[{"id":"primary","name":str(option["label"]),"unit":str(option.get("unit","value")),"group":"Analysed parameter","values":values}]
    for field_id in timeline_ids:
        if field_id not in option_lookup: continue
        timeline_option=option_lookup[field_id]
        if option.get("type")=="raw" and field_id==f"raw::{option.get('column','')}":
            continue
        vals=terrain_cache.get(field_id)
        if vals is None and timeline_option.get("type")=="raw":
            try:
                raw_index=int(timeline_option.get("column_index")) if timeline_option.get("column_index") is not None else None
            except Exception:
                raw_index=None
            if raw_index is None or not (0 <= raw_index < len(data["header"])):
                raw_index=_best_numeric_col_index(data["header"],[r["row"] for r in records],[str(timeline_option.get("column", ""))])
            vals=[_parse_float(_clean_cell(r["row"],raw_index)) for r in records]
        if vals is None:
            field=timeline_field_map.get(field_id); vals=[r.get(field) if field else None for r in records]
        if timeline_option["label"]==option["label"]: continue
        series.append({"id":field_id,"name":timeline_option["label"],"unit":timeline_option["unit"],"group":timeline_option["group"],"values":vals})
    x_values=[r["plot_time"] for r in records]
    bands=[{"x0":e["start_x"],"x1":e["end_x"],"class":e["class"]} for e in episodes]
    return {
        "source":os.path.basename(csv_path),"summary":summary,"route_segments":route_segments,"popup_points":popup_points,
        "episodes":episodes,"verdicts":verdicts,"timeline":{"x":x_values,"series":series,"bands":bands},
        "primary":{"id":option.get("id"),"label":option.get("label"),"unit":option.get("unit","value"),"config":config,"source_note":source_note},
        "privacy_enabled":bool((privacy_config or {}).get("enabled")),
    }

def _analysis_tile_layers_js(initial_tile_key: str) -> Tuple[str, str]:
    ordered = _ordered_tile_keys(initial_tile_key, ALL_TILE_KEYS)
    lines: List[str] = []
    names: List[str] = []
    for n, key in enumerate(ordered):
        provider = TILE_PROVIDERS[key]
        var_name = f"baseLayer{n}"
        options = dict(provider.get("options", {}))
        options.pop("referrerPolicy", None)
        lines.append(f"const {var_name} = L.tileLayer({json.dumps(provider['url'])}, {json.dumps(options)});")
        names.append(f"{json.dumps(provider['name'])}: {var_name}")
    lines.append("baseLayer0.addTo(map);")
    return "\n".join(lines), "{" + ",".join(names) + "}"


def build_flight_analysis_html(payload: Dict[str, Any], initial_tile_key: str = BUILTIN_PRESET_INITIAL_TILE_KEY, graph_export: Optional[Dict[str, Any]] = None) -> str:
    tile_js, base_layers_js = _analysis_tile_layers_js(initial_tile_key)
    summary = payload["summary"]; primary = payload["primary"]; config = primary["config"]
    graph_export = dict(graph_export or {})
    png_width = max(640, min(7680, int(graph_export.get("width", 1920) or 1920)))
    png_height = max(360, min(4320, int(graph_export.get("height", 1080) or 1080)))
    default_chart_title = f"{primary.get('label', 'Flight parameter')} and selected telemetry over time"
    chart_title = str(graph_export.get("title") or default_chart_title).strip()
    default_png_name = f"{os.path.splitext(str(payload.get('source') or 'flight'))[0]} - {primary.get('label', 'analysis')} timeline"
    png_filename = str(graph_export.get("filename") or default_png_name).strip()
    png_filename = re.sub(r'[<>:"/\\|?*]+', '_', png_filename).strip(' .') or "flight_parameter_analysis"
    unit = str(primary.get("unit") or "value")
    def fmt_metric(value: Any, decimals: int = 2) -> str:
        if value is None: return "n/a"
        return f"{float(value):.{decimals}f} {unit}".strip()
    verdict_rows = "".join(
        f'<div class="verdict-row"><strong>{escape(v["name"])}</strong><span class="badge {"pass" if v["badge"] in ("Mostly OK","Mostly good") else "fail" if v["badge"] in ("Not OK","Attention") else "warn"}">{escape(v["badge"])}</span><span>{escape(v["text"])}</span></div>'
        for v in payload["verdicts"]
    )
    subtitle = f"Source: {escape(payload['source'])} · Route colour and flagged episodes analyse {escape(str(primary['label']))}."
    if payload.get("privacy_enabled"): subtitle += " Privacy trimming was applied before GPS coordinates were embedded."
    threshold_text = (
        f"Higher is better: good ≥ {config['good_threshold']:g}, bad < {config['bad_threshold']:g}." if config.get("rule")=="higher" else
        f"Lower is better: good ≤ {config['good_threshold']:g}, bad > {config['bad_threshold']:g}." if config.get("rule")=="lower" else
        f"Value bands: low < {min(config['good_threshold'],config['bad_threshold']):g}, high ≥ {max(config['good_threshold'],config['bad_threshold']):g}."
    )
    template=r'''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><meta name="referrer" content="no-referrer-when-downgrade"><title>Flight data analysis: __PRIMARY_LABEL__</title><link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"><style>
:root{color-scheme:dark;--bg:#0f172a;--panel:#172033;--panel2:#202b40;--text:#e8edf5;--muted:#aebbd0;--border:#34425d;--good:#22c55e;--warn:#f59e0b;--bad:#ef4444;--missing:#94a3b8}*{box-sizing:border-box}body{margin:0;font-family:Inter,ui-sans-serif,system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;background:var(--bg);color:var(--text)}header{padding:18px 22px 10px;max-width:1500px;margin:auto}h1{margin:0 0 5px;font-size:clamp(1.35rem,3vw,2rem)}h2{font-size:1.05rem;margin:0 0 10px}.sub{color:var(--muted);margin:0;line-height:1.45}.layout{max-width:1500px;margin:auto;padding:12px 22px 28px;display:grid;grid-template-columns:minmax(0,1.7fr) minmax(320px,.8fr);gap:16px}.panel{background:var(--panel);border:1px solid var(--border);border-radius:14px;overflow:hidden}#map{height:610px;width:100%;background:#cbd5e1}.side{padding:15px;display:grid;gap:14px;align-content:start}.metrics{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:9px}.metric{background:var(--panel2);border:1px solid var(--border);border-radius:10px;padding:10px}.metric b{display:block;font-size:1.18rem;margin-top:3px;overflow-wrap:anywhere}.metric span{color:var(--muted);font-size:.82rem}.verdict{border-radius:12px;border:1px solid var(--border);padding:12px;background:var(--panel2)}.verdict-grid{display:grid;gap:8px}.verdict-row{display:grid;grid-template-columns:78px 96px 1fr;gap:8px;align-items:start;font-size:.9rem}.badge{display:inline-block;border-radius:999px;padding:3px 8px;font-weight:700;font-size:.75rem;text-align:center}.pass{background:rgba(34,197,94,.16);color:#86efac}.fail{background:rgba(239,68,68,.16);color:#fca5a5}.warn{background:rgba(245,158,11,.16);color:#fcd34d}.legend{background:rgba(15,23,42,.94);color:white;border:1px solid rgba(255,255,255,.25);padding:9px 11px;border-radius:8px;line-height:1.5;box-shadow:0 2px 10px rgba(0,0,0,.25);max-width:270px}.swatch{display:inline-block;width:25px;height:4px;margin-right:7px;vertical-align:middle}.events{overflow:auto;max-height:290px}table{width:100%;border-collapse:collapse;font-size:.82rem}th,td{padding:7px 5px;border-bottom:1px solid var(--border);text-align:left;white-space:nowrap}th{color:var(--muted);font-weight:600}tbody tr{cursor:pointer}tbody tr:hover{background:rgba(56,189,248,.1)}.chart-panel{grid-column:1/-1;padding:10px 10px 4px}.chart-head{display:flex;gap:10px;align-items:center;justify-content:space-between;flex-wrap:wrap;padding:2px 2px 8px}.export-png{border:1px solid #475569;background:#243149;color:#e8edf5;border-radius:8px;padding:8px 12px;font-weight:650;cursor:pointer}.export-png:hover{background:#31415f}#timeline{min-height:650px}.method{grid-column:1/-1;padding:14px 16px;color:var(--muted);font-size:.9rem;line-height:1.5}.method strong{color:var(--text)}.leaflet-popup-content-wrapper,.leaflet-popup-tip{background:#172033;color:#e8edf5}@media(max-width:950px){.layout{grid-template-columns:1fr;padding:10px}#map{height:500px}.chart-panel,.method{grid-column:auto}}@media(max-width:520px){header{padding:14px 12px 6px}.metrics{grid-template-columns:1fr 1fr}.verdict-row{grid-template-columns:68px 78px 1fr}#map{height:420px}}
</style></head><body><header><h1>Flight data analysis: __PRIMARY_LABEL__</h1><p class="sub">__SUBTITLE__</p></header><main class="layout"><section class="panel"><div id="map"></div></section><aside class="panel side"><section><h2>Flight summary</h2><div class="metrics">
<div class="metric"><span>Recorded duration</span><b>__DURATION__</b></div><div class="metric"><span>Track distance</span><b>__DISTANCE__</b></div><div class="metric"><span>Maximum speed</span><b>__MAXSPEED__</b></div><div class="metric"><span>Maximum relative altitude</span><b>__MAXALT__</b></div><div class="metric"><span>Parameter minimum</span><b>__PMIN__</b></div><div class="metric"><span>Parameter average</span><b>__PAVG__</b></div><div class="metric"><span>Parameter maximum</span><b>__PMAX__</b></div><div class="metric"><span>Flagged time</span><b>__FLAGGED__</b><span>__FLAGPCT__% of analysed time · __RUNS__ episode(s)</span></div>
</div></section><section class="verdict"><h2>Deterministic findings</h2><div class="verdict-grid">__VERDICTS__</div></section><section class="events"><h2>Flagged episodes</h2><table><thead><tr><th>Time</th><th>Class</th><th>Duration</th><th>Minimum (__UNIT__)</th><th>Average (__UNIT__)</th><th>Maximum (__UNIT__)</th></tr></thead><tbody id="events-body"></tbody></table></section></aside><section class="panel chart-panel"><div class="chart-head"><h2>Timeline: analysed parameter and selected supporting data</h2><button class="export-png" id="export-timeline-png" type="button">Export __PNG_WIDTH__ × __PNG_HEIGHT__ PNG</button></div><div id="timeline"></div></section><section class="panel method"><strong>How this analysis works:</strong> __THRESHOLDS__ The flight path is blue for __GOODLABEL__, orange for __WARNLABEL__, red dashed for __BADLABEL__, and grey when the selected parameter is unavailable. Valid GPS samples below 4 satellites or missing coordinates break the route. Timeline bands mark warning and bad episodes. Source: __SOURCE_NOTE__</section></main>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script><script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script><script>
const routeSegments=__ROUTES__,popupPoints=__POINTS__,episodes=__EPISODES__,timeline=__TIMELINE__,primary=__PRIMARY__,graphTitle=__GRAPH_TITLE__,pngFilename=__PNG_FILENAME__,pngWidth=__PNG_WIDTH__,pngHeight=__PNG_HEIGHT__;
const map=L.map("map",{preferCanvas:true});__TILEJS__
const groups={good:L.layerGroup().addTo(map),warn:L.layerGroup().addTo(map),bad:L.layerGroup().addTo(map),missing:L.layerGroup().addTo(map)},points=L.layerGroup(),bounds=[];
const styles={good:{color:"#22c55e",weight:4,opacity:.92},warn:{color:"#f59e0b",weight:5,opacity:.95},bad:{color:"#ef4444",weight:6,opacity:.95,dashArray:"9 6"},missing:{color:"#94a3b8",weight:3,opacity:.75,dashArray:"3 7"}};
const labels={good:primary.config.good_label,warn:primary.config.warn_label,bad:primary.config.bad_label,missing:"No parameter value"};
routeSegments.forEach(seg=>{const cls=seg.class||"missing",line=L.polyline(seg.coords,styles[cls]||styles.missing);line.bindPopup(`<b>${primary.label}</b><br>${labels[cls]}`);line.addTo(groups[cls]||groups.missing);seg.coords.forEach(c=>bounds.push(c));});if(bounds.length)map.fitBounds(bounds,{padding:[24,24]});
if(routeSegments.length){const first=routeSegments.find(s=>s.coords.length),last=[...routeSegments].reverse().find(s=>s.coords.length);if(first)L.marker(first.coords[0]).addTo(map).bindPopup("<b>Start of visible track</b>");if(last)L.marker(last.coords[last.coords.length-1]).addTo(map).bindPopup("<b>End of visible track</b>");}
const fmt=(v,d=2)=>v===null||v===undefined||Number.isNaN(Number(v))?"n/a":Number(v).toFixed(d);popupPoints.forEach(p=>{const cls=p.class||"missing",c=styles[cls].color,m=L.circleMarker([p.lat,p.lon],{radius:cls==="bad"?4:2.7,color:c,fillColor:c,fillOpacity:.75,weight:1});m.bindPopup(`<b>${p.time}</b><br><b>${primary.label}:</b> ${fmt(p.value)} ${primary.unit}<br>Class: ${labels[cls]}<br>Satellites: ${fmt(p.sats,0)}<br>Logged speed: ${fmt(p.speed,1)} km/h<br>Coordinate speed: ${fmt(p.coord_speed,1)} km/h<br>Relative altitude: ${fmt(p.alt,1)} m<br>RQly: ${fmt(p.rqly,0)}%<br>RSNR: ${fmt(p.rsnr,1)} dB<br>RSSI: ${fmt(p.rssi,1)} dBm<br>Power: ${fmt(p.power,1)} W`);m.addTo(points)});
L.control.layers(__BASELAYERS__,{[primary.config.good_label]:groups.good,[primary.config.warn_label]:groups.warn,[primary.config.bad_label]:groups.bad,"No parameter value":groups.missing,"Inspection points":points},{collapsed:true}).addTo(map);L.control.scale({metric:true,imperial:false}).addTo(map);
const legend=L.control({position:"bottomright"});legend.onAdd=function(){const d=L.DomUtil.create("div","legend");d.innerHTML=`<b>${primary.label}</b><br><span class="swatch" style="background:#22c55e"></span>${primary.config.good_label}<br><span class="swatch" style="background:#f59e0b"></span>${primary.config.warn_label}<br><span class="swatch" style="background:#ef4444"></span>${primary.config.bad_label}<br><span class="swatch" style="background:#94a3b8"></span>No value`;return d};legend.addTo(map);
const body=document.getElementById("events-body");episodes.forEach(e=>{const tr=document.createElement("tr");tr.innerHTML=`<td>${e.start_time}–${e.end_time}</td><td>${labels[e.class]||e.class}</td><td>${Number(e.duration_s).toFixed(1)} s</td><td>${fmt(e.min_value)}</td><td>${fmt(e.avg_value)}</td><td>${fmt(e.max_value)}</td>`;tr.onclick=()=>{if(e.coords&&e.coords.length)map.fitBounds(e.coords,{padding:[80,80],maxZoom:18})};body.appendChild(tr)});if(!episodes.length)body.innerHTML='<tr><td colspan="6">No warning or bad episodes were found with the chosen thresholds.</td></tr>';
const groupOrder=[];timeline.series.forEach(s=>{if(!groupOrder.includes(s.group))groupOrder.push(s.group)});const traces=[];const layout={title:{text:graphTitle,x:.5,xanchor:"center",font:{size:20}},paper_bgcolor:"#172033",plot_bgcolor:"#172033",font:{color:"#e8edf5"},margin:{l:72,r:34,t:76,b:70},hovermode:"x unified",legend:{orientation:"h",y:1.04},xaxis:{title:"Local time",type:"date",tickformat:"%H:%M:%S",hoverformat:"%H:%M:%S.%L",nticks:12,tickangle:0,automargin:true,gridcolor:"#34425d"},shapes:[]};
const n=Math.max(1,groupOrder.length),gap=.035,rowH=(1-gap*(n-1))/n;groupOrder.forEach((group,gi)=>{const axis=gi===0?"y":"y"+(gi+1),key=gi===0?"yaxis":"yaxis"+(gi+1),top=1-gi*(rowH+gap),bottom=top-rowH;const first=timeline.series.find(s=>s.group===group);layout[key]={title:first?first.unit:"value",domain:[bottom,top],gridcolor:"#34425d",zerolinecolor:"#34425d",automargin:true};timeline.series.filter(s=>s.group===group).forEach(s=>traces.push({x:timeline.x,y:s.values,name:s.name,type:"scattergl",mode:"lines",connectgaps:false,yaxis:axis,line:{width:s.id==="coord_speed"?1.2:2,dash:s.id==="coord_speed"?"dot":"solid"}}))});timeline.bands.forEach(b=>layout.shapes.push({type:"rect",xref:"x",yref:"paper",x0:b.x0,x1:b.x1,y0:0,y1:1,fillcolor:b.class==="bad"?"rgba(239,68,68,.17)":"rgba(245,158,11,.13)",line:{width:0},layer:"below"}));document.getElementById("timeline").style.height=Math.max(430,groupOrder.length*190)+"px";Plotly.newPlot("timeline",traces,layout,{responsive:true,displaylogo:false,toImageButtonOptions:{format:"png",filename:pngFilename,width:pngWidth,height:pngHeight,scale:1}});document.getElementById("export-timeline-png").addEventListener("click",()=>Plotly.downloadImage("timeline",{format:"png",filename:pngFilename,width:pngWidth,height:pngHeight,scale:1}));
</script></body></html>'''
    flagged=float(summary.get("warn_seconds") or 0)+float(summary.get("bad_seconds") or 0)
    flagpct=float(summary.get("warn_pct") or 0)+float(summary.get("bad_pct") or 0)
    repl={
        "__PRIMARY_LABEL__":escape(str(primary["label"])),"__UNIT__":escape(unit),"__SUBTITLE__":subtitle,"__DURATION__":_format_analysis_duration(summary.get("duration_s")),
        "__DISTANCE__":f"{float(summary.get('distance_km') or 0):.2f} km","__MAXSPEED__":"n/a" if summary.get("max_speed") is None else f"{float(summary['max_speed']):.1f} km/h",
        "__MAXALT__":"n/a" if summary.get("max_alt") is None else f"{float(summary['max_alt']):.1f} m","__PMIN__":fmt_metric(summary.get("metric_min")),
        "__PAVG__":fmt_metric(summary.get("metric_avg")),"__PMAX__":fmt_metric(summary.get("metric_max")),"__FLAGGED__":_format_analysis_duration(flagged),
        "__FLAGPCT__":f"{flagpct:.1f}","__RUNS__":str(int(summary.get("flagged_runs") or 0)),"__VERDICTS__":verdict_rows,
        "__THRESHOLDS__":escape(threshold_text),"__GOODLABEL__":escape(str(config["good_label"])),"__WARNLABEL__":escape(str(config["warn_label"])),
        "__BADLABEL__":escape(str(config["bad_label"])),"__SOURCE_NOTE__":escape(str(primary.get("source_note") or "")),
        "__ROUTES__":json.dumps(payload["route_segments"],separators=(",",":")),"__POINTS__":json.dumps(payload["popup_points"],separators=(",",":")),
        "__EPISODES__":json.dumps(payload["episodes"],separators=(",",":")),"__TIMELINE__":json.dumps(payload["timeline"],separators=(",",":")),
        "__PRIMARY__":json.dumps(primary,separators=(",",":")),"__GRAPH_TITLE__":json.dumps(chart_title),"__PNG_FILENAME__":json.dumps(png_filename),"__PNG_WIDTH__":str(png_width),"__PNG_HEIGHT__":str(png_height),"__TILEJS__":tile_js,"__BASELAYERS__":base_layers_js,
    }
    for key,value in repl.items(): template=template.replace(key,str(value))
    return template

def output_path_for_analysis_report(csv_path: str, primary_metric: Optional[Dict[str, Any]] = None) -> str:
    """Use the analysed parameter in the filename so separate analyses do not overwrite each other."""
    base = os.path.splitext(csv_path)[0]
    if not primary_metric:
        return base + " (flight analysis).html"
    safe = str(primary_metric.get("short") or primary_metric.get("label") or "parameter")
    safe = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in safe).strip("_") or "parameter"
    return f"{base} (analysis {safe}).html"


def process_flight_analysis_report(csv_path: str, timeline_ids: List[str], initial_tile_key: str = BUILTIN_PRESET_INITIAL_TILE_KEY, privacy_config: Optional[Dict[str, Any]] = None, primary_metric: Optional[Dict[str, Any]] = None, analysis_config: Optional[Dict[str, Any]] = None, status_callback: Optional[Any] = None) -> str:
    payload = build_flight_analysis_payload(csv_path, timeline_ids, privacy_config=privacy_config, primary_metric=primary_metric, analysis_config=analysis_config, status_callback=status_callback)
    payload_primary = payload.get("primary") if isinstance(payload.get("primary"), dict) else None
    out_path = output_path_for_analysis_report(csv_path, primary_metric or payload_primary)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(build_flight_analysis_html(payload, initial_tile_key=initial_tile_key))
    return out_path


# ---------------------------------------------------------------------------
# Local terrain database support (v31)
# ---------------------------------------------------------------------------
# The database cache is intentionally process-local. A broad terrain root such as
# Downloads is recursively indexed once per app session, then reused by every tab.
_TERRAIN_DB_CACHE: Dict[str, Any] = {}


def _terrain_choice_normalized(value: Any) -> str:
    s = str(value or "").strip().lower()
    if "local first" in s or "fallback" in s:
        return "local_then_online"
    if "open" in s or "online" in s:
        return "online"
    return "local"


def _terrain_settings() -> Dict[str, Any]:
    settings = load_parameter_settings()
    raw = str(settings.get("terrain_folder", "") or "").strip()
    return {
        "choice": _terrain_choice_normalized(settings.get("terrain_source", "Local terrain files")),
        "folder": normalize_path(raw) if raw else "",
    }


def _terrain_filename_stem(path: str) -> str:
    """Return a terrain filename stem while tolerating duplicate-download suffixes."""
    name = os.path.basename(path).strip()
    lower = name.lower()
    for suffix in (".dat", ".hgt"):
        if lower.endswith(suffix):
            return name[: -len(suffix)]
    return os.path.splitext(name)[0]


def _parse_terrain_tile_name(path: str) -> Optional[Tuple[int, int]]:
    """Parse names such as N49W113.DAT, including harmless suffixes like ' (1)'."""
    stem = _terrain_filename_stem(path).upper()
    match = re.match(r"^([NS])(\d{1,2})([EW])(\d{1,3})(?:$|[^0-9].*)", stem)
    if not match:
        return None
    lat_deg = int(match.group(2)) * (1 if match.group(1) == "N" else -1)
    lon_deg = int(match.group(4)) * (1 if match.group(3) == "E" else -1)
    return lat_deg, lon_deg


def _terrain_tile_basename(key: Tuple[int, int], extension: str = "DAT") -> str:
    lat_deg, lon_deg = key
    ns = "N" if lat_deg >= 0 else "S"
    ew = "E" if lon_deg >= 0 else "W"
    return f"{ns}{abs(int(lat_deg)):02d}{ew}{abs(int(lon_deg)):03d}.{extension.upper()}"


class _ArduPilotDatTile:
    """Read ArduPilot terrain DAT files, including older version_minor=0 files."""

    BLOCK = 2048
    HOFF = 22
    HCOUNT = 28 * 32
    MOFF = 22 + HCOUNT * 2
    SPATIAL_BIN_DEG = 0.05

    def __init__(self, path: str):
        self.path = path
        self.blocks: Dict[Tuple[int, int], List[Dict[str, Any]]] = {}
        self.spatial_bins: Dict[Tuple[int, int], List[Dict[str, Any]]] = {}
        self.spacings: set[int] = set()
        self.minor_versions: set[int] = set()
        self.latdeg: Optional[int] = None
        self.londeg: Optional[int] = None
        self.cache: Dict[int, Tuple[int, ...]] = {}
        self.valid_block_count = 0
        self._index()

    @classmethod
    def _block_count(cls, size: int) -> int:
        if size >= cls.BLOCK and size % cls.BLOCK == 0:
            return size // cls.BLOCK
        # Some valid files can have a final 1821-byte data block without padding.
        data_size = cls.MOFF + 7
        if size >= data_size and (size - data_size) % cls.BLOCK == 0:
            return ((size - data_size) // cls.BLOCK) + 1
        raise ValueError("invalid ArduPilot DAT size")

    def _index(self) -> None:
        size = os.path.getsize(self.path)
        total_blocks = self._block_count(size)
        with open(self.path, "rb") as file_obj:
            for block_number in range(total_blocks):
                offset = block_number * self.BLOCK
                file_obj.seek(offset)
                raw = file_obj.read(min(self.BLOCK, size - offset))
                if len(raw) < self.MOFF + 7:
                    continue
                try:
                    bitmap, lat_i, lon_i, _crc, version, spacing = struct.unpack_from("<QiiHHH", raw, 0)
                    grid_x, grid_y, lon_deg = struct.unpack_from("<HHh", raw, self.MOFF)
                    lat_deg = struct.unpack_from("<b", raw, self.MOFF + 6)[0]
                    # Older, valid files commonly contain 0 here. This byte was added in a
                    # backwards-compatible trailer and must not be used to reject the tile.
                    minor = struct.unpack_from("<B", raw, self.MOFF + 7)[0] if len(raw) > self.MOFF + 7 else 0
                except struct.error:
                    continue
                if version != 1 or not (1 <= int(spacing) <= 1000):
                    continue
                block = {
                    "off": offset,
                    "lat": float(lat_i) / 1.0e7,
                    "lon": float(lon_i) / 1.0e7,
                    "sp": float(spacing),
                    "bitmap": int(bitmap),
                    "grid_x": int(grid_x),
                    "grid_y": int(grid_y),
                    "minor": int(minor),
                }
                self.latdeg = int(lat_deg)
                self.londeg = int(lon_deg)
                self.spacings.add(int(spacing))
                self.minor_versions.add(int(minor))
                self.blocks.setdefault((int(grid_x), int(grid_y)), []).append(block)
                bin_key = (
                    math.floor(block["lat"] / self.SPATIAL_BIN_DEG),
                    math.floor(block["lon"] / self.SPATIAL_BIN_DEG),
                )
                self.spatial_bins.setdefault(bin_key, []).append(block)
                self.valid_block_count += 1
        if not self.valid_block_count:
            raise ValueError("no valid terrain blocks (major version 1) were found")

    @staticmethod
    def _north_east(sw_lat: float, sw_lon: float, lat: float, lon: float) -> Tuple[float, float]:
        """Small-area north/east displacement in metres from the stored block origin."""
        north = math.radians(lat - sw_lat) * 6371000.0
        east = math.radians(lon - sw_lon) * 6371000.0 * math.cos(math.radians((lat + sw_lat) * 0.5))
        return north, east

    def _heights(self, offset: int) -> Tuple[int, ...]:
        if offset in self.cache:
            return self.cache[offset]
        with open(self.path, "rb") as file_obj:
            file_obj.seek(offset + self.HOFF)
            raw = file_obj.read(self.HCOUNT * 2)
        if len(raw) != self.HCOUNT * 2:
            raise ValueError("truncated terrain height block")
        values = struct.unpack("<" + "h" * self.HCOUNT, raw)
        if len(self.cache) > 96:
            self.cache.pop(next(iter(self.cache)))
        self.cache[offset] = values
        return values

    def _candidate_blocks(self, lat: float, lon: float) -> List[Dict[str, Any]]:
        candidates: List[Dict[str, Any]] = []
        seen: set[int] = set()

        def add(items: List[Dict[str, Any]]) -> None:
            for item in items:
                marker = int(item["off"])
                if marker not in seen:
                    seen.add(marker)
                    candidates.append(item)

        # Fast lookup by ArduPilot grid index.
        if self.latdeg is not None and self.londeg is not None:
            north = math.radians(lat - self.latdeg) * 6371000.0
            east = math.radians(lon - self.londeg) * 6371000.0 * math.cos(math.radians(lat))
            for spacing in self.spacings:
                grid_x = int(math.floor(north / max(1.0, 24.0 * spacing)))
                grid_y = int(math.floor(east / max(1.0, 28.0 * spacing)))
                for dx in range(-3, 4):
                    for dy in range(-3, 4):
                        add(self.blocks.get((grid_x + dx, grid_y + dy), []))

        # Robust fallback based on the actual block origins stored in the file. This also
        # tolerates old pre-version_minor files whose coordinate arithmetic differs slightly.
        bin_lat = math.floor(lat / self.SPATIAL_BIN_DEG)
        bin_lon = math.floor(lon / self.SPATIAL_BIN_DEG)
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                add(self.spatial_bins.get((bin_lat + dx, bin_lon + dy), []))
        return candidates

    def elevation(self, lat: float, lon: float) -> Optional[float]:
        candidates: List[Tuple[float, Dict[str, Any], float, float]] = []
        for block in self._candidate_blocks(lat, lon):
            north, east = self._north_east(block["lat"], block["lon"], lat, lon)
            spacing = float(block["sp"])
            tolerance = max(5.0, spacing * 0.25)
            if -tolerance <= north <= 27.0 * spacing + tolerance and -tolerance <= east <= 31.0 * spacing + tolerance:
                # Prefer the finest grid, then the closest block origin.
                candidates.append((spacing * 1_000_000.0 + abs(north) + abs(east), block, north, east))
        if not candidates:
            return None
        _, block, north, east = min(candidates, key=lambda item: item[0])
        spacing = float(block["sp"])
        x = max(0.0, min(27.0, north / spacing))
        y = max(0.0, min(31.0, east / spacing))
        ix = min(26, max(0, int(math.floor(x))))
        iy = min(30, max(0, int(math.floor(y))))
        frac_x = x - ix
        frac_y = y - iy
        heights = self._heights(int(block["off"]))

        def at(x_index: int, y_index: int) -> float:
            return float(heights[x_index * 32 + y_index])

        corners = (at(ix, iy), at(ix + 1, iy), at(ix, iy + 1), at(ix + 1, iy + 1))
        if any(value <= -32000 for value in corners):
            return None
        west = (1.0 - frac_x) * corners[0] + frac_x * corners[1]
        east_value = (1.0 - frac_x) * corners[2] + frac_x * corners[3]
        return (1.0 - frac_y) * west + frac_y * east_value

    @property
    def resolution_m(self) -> float:
        return float(min(self.spacings))

    @property
    def description(self) -> str:
        return f"ArduPilot DAT ({'/'.join(map(str, sorted(self.spacings)))} m grid)"


class _SrtmHgtTile:
    def __init__(self, path: str):
        self.path = path
        parsed = _parse_terrain_tile_name(path)
        if not parsed:
            raise ValueError("unrecognised HGT filename")
        self.lat0, self.lon0 = parsed
        size = os.path.getsize(path)
        self.n = int(round(math.sqrt(size / 2)))
        if self.n * self.n * 2 != size or self.n < 2:
            raise ValueError("invalid HGT dimensions")

    def elevation(self, lat: float, lon: float) -> Optional[float]:
        if not (self.lat0 <= lat <= self.lat0 + 1 and self.lon0 <= lon <= self.lon0 + 1):
            return None
        row = (self.lat0 + 1 - lat) * (self.n - 1)
        column = (lon - self.lon0) * (self.n - 1)
        row_index = min(self.n - 2, max(0, int(math.floor(row))))
        column_index = min(self.n - 2, max(0, int(math.floor(column))))
        row_fraction = row - row_index
        column_fraction = column - column_index
        indices = [
            row_index * self.n + column_index,
            row_index * self.n + column_index + 1,
            (row_index + 1) * self.n + column_index,
            (row_index + 1) * self.n + column_index + 1,
        ]
        values: List[int] = []
        with open(self.path, "rb") as file_obj:
            for index in indices:
                file_obj.seek(index * 2)
                raw = file_obj.read(2)
                if len(raw) != 2:
                    return None
                values.append(struct.unpack(">h", raw)[0])
        if any(value == -32768 for value in values):
            return None
        return (1.0 - row_fraction) * (
            (1.0 - column_fraction) * values[0] + column_fraction * values[1]
        ) + row_fraction * (
            (1.0 - column_fraction) * values[2] + column_fraction * values[3]
        )

    @property
    def resolution_m(self) -> float:
        return 30.87 * 3600.0 / float(self.n - 1)

    @property
    def description(self) -> str:
        return f"SRTM HGT ({3600 / (self.n - 1):.0f} arc-second grid)"


class _LocalTerrainDatabase:
    """Recursively open and index every readable local terrain file once per app session.

    ArduPilot DAT coordinates are read from the file's internal metadata, not trusted from
    its filename. This means a DAT file remains usable if a user renames it while keeping
    the .DAT extension. Standard HGT files do not contain their own geographic position,
    so their N/S/E/W coordinate filename is still required by the HGT format.
    """

    def __init__(self, folder: str):
        self.folder = folder
        self.tiles_by_key: Dict[Tuple[int, int], List[Any]] = {}
        self.all_tiles: List[Any] = []
        self.folder_count = 0
        self.detected_file_count = 0
        self.loaded_file_count = 0
        self.rejected_file_count = 0
        self.load_errors: List[str] = []
        self.renamed_dat_count = 0
        self._use_count = 0
        self._scan()

    @staticmethod
    def _dat_keys(tile: _ArduPilotDatTile) -> List[Tuple[int, int]]:
        keys: set[Tuple[int, int]] = set()
        if tile.latdeg is not None and tile.londeg is not None:
            keys.add((int(tile.latdeg), int(tile.londeg)))
        # Use actual internal block origins as a second independent geographic index.
        for blocks in tile.blocks.values():
            for block in blocks:
                keys.add((math.floor(float(block["lat"])), math.floor(float(block["lon"]))))
        return sorted(keys)

    def _scan(self) -> None:
        if not os.path.isdir(self.folder):
            raise ValueError("the selected terrain folder does not exist")
        for root, _directories, names in os.walk(self.folder):
            self.folder_count += 1
            for name in names:
                extension = os.path.splitext(name)[1].lower()
                if extension not in (".dat", ".hgt"):
                    continue
                self.detected_file_count += 1
                path = os.path.join(root, name)
                try:
                    if extension == ".dat":
                        tile = _ArduPilotDatTile(path)
                        keys = self._dat_keys(tile)
                        named_key = _parse_terrain_tile_name(path)
                        if named_key is None or named_key not in keys:
                            self.renamed_dat_count += 1
                    else:
                        # HGT has no embedded location metadata; its standard tile name is
                        # the only reliable way to know where its samples belong.
                        tile = _SrtmHgtTile(path)
                        parsed = _parse_terrain_tile_name(path)
                        keys = [parsed] if parsed is not None else []
                    if not keys:
                        raise ValueError("terrain location could not be determined")
                    self.all_tiles.append(tile)
                    for key in keys:
                        self.tiles_by_key.setdefault(key, []).append(tile)
                    self.loaded_file_count += 1
                except Exception as exc:
                    self.rejected_file_count += 1
                    if len(self.load_errors) < 50:
                        self.load_errors.append(f"{path}: {exc}")

        self.all_tiles.sort(key=lambda tile: (float(tile.resolution_m), tile.path.lower()))
        for tiles in self.tiles_by_key.values():
            tiles.sort(key=lambda tile: (float(tile.resolution_m), tile.path.lower()))
        if not self.all_tiles:
            details = "; ".join(self.load_errors[:3])
            suffix = f" First read error(s): {details}" if details else ""
            raise ValueError(
                "no readable ArduPilot .DAT or standard N/S/E/W-named SRTM .HGT terrain files "
                f"were found recursively.{suffix}"
            )

    def use_message(self) -> str:
        self._use_count += 1
        if self._use_count == 1:
            renamed_note = (
                f" {self.renamed_dat_count} DAT file(s) were indexed by embedded coordinates rather than filename."
                if self.renamed_dat_count else ""
            )
            rejected_note = (
                f" {self.rejected_file_count} file(s) were skipped because they were not readable terrain data."
                if self.rejected_file_count else ""
            )
            return (
                f"Recursive terrain database loaded for this app session: scanned {self.folder_count} folder(s) "
                f"under {self.folder}; detected {self.detected_file_count} DAT/HGT file(s) and opened/indexed "
                f"{self.loaded_file_count}. Every readable terrain file is opened during this first scan."
                f"{renamed_note}{rejected_note}"
            )
        return (
            f"Reusing fully loaded recursive terrain database for this app session: "
            f"{self.loaded_file_count} readable tile file(s) under {self.folder}."
        )

    @staticmethod
    def _tile_source(tile: Any) -> str:
        return f"{tile.description}: {os.path.basename(tile.path)}"

    def elevation(self, lat: float, lon: float) -> Tuple[Optional[float], str]:
        main_key = (math.floor(lat), math.floor(lon))
        search_keys = [main_key]
        # Terrain grids can overlap slightly at degree boundaries.
        for lat_offset in (-1, 0, 1):
            for lon_offset in (-1, 0, 1):
                key = (main_key[0] + lat_offset, main_key[1] + lon_offset)
                if key not in search_keys:
                    search_keys.append(key)

        tried: set[int] = set()
        local_candidates: List[Any] = []
        for key in search_keys:
            for tile in self.tiles_by_key.get(key, []):
                marker = id(tile)
                if marker not in tried:
                    tried.add(marker)
                    local_candidates.append(tile)
        local_candidates.sort(key=lambda tile: (float(tile.resolution_m), tile.path.lower()))

        for tile in local_candidates:
            try:
                value = tile.elevation(lat, lon)
            except Exception:
                value = None
            if value is not None:
                return float(value), self._tile_source(tile)

        # Independent fallback: check every already-open tile, ignoring its filename and
        # primary degree index. This catches renamed/misnamed DAT files and unusual boundary
        # metadata without rescanning the disk or silently substituting guessed terrain.
        fallback_count = 0
        for tile in self.all_tiles:
            marker = id(tile)
            if marker in tried:
                continue
            fallback_count += 1
            try:
                value = tile.elevation(lat, lon)
            except Exception:
                value = None
            if value is not None:
                return float(value), self._tile_source(tile) + " (full-database fallback match)"

        expected_dat = _terrain_tile_basename(main_key, "DAT")
        expected_hgt = _terrain_tile_basename(main_key, "HGT")
        nearby_names = sorted({os.path.basename(tile.path) for tile in local_candidates})
        detail = (
            f" Nearby indexed candidate(s): {', '.join(nearby_names[:6])}."
            if nearby_names else " No tile was indexed to this degree."
        )
        read_error_note = (
            " Some detected files could not be read; first error: " + self.load_errors[0]
            if self.load_errors else ""
        )
        return None, (
            f"No loaded local terrain grid covered {lat:.6f}, {lon:.6f}. The app checked the expected "
            f"{expected_dat}/{expected_hgt} degree, neighbouring degrees, and then all {fallback_count} "
            f"remaining loaded tile(s).{detail}{read_error_note}"
        )

def _terrain_cache_key(folder: str) -> str:
    return os.path.normcase(os.path.abspath(folder))


def _get_local_terrain_database(folder: str) -> _LocalTerrainDatabase:
    """Return a fully opened recursive terrain database cached until the app process closes."""
    key = _terrain_cache_key(folder)
    database = _TERRAIN_DB_CACHE.get(key)
    if database is None:
        database = _LocalTerrainDatabase(os.path.abspath(folder))
        _TERRAIN_DB_CACHE[key] = database
    return database


def _clear_local_terrain_database_cache(folder: Optional[str] = None) -> None:
    if folder:
        _TERRAIN_DB_CACHE.pop(_terrain_cache_key(folder), None)
    else:
        _TERRAIN_DB_CACHE.clear()


def _query_local_terrain_point(lat: float, lon: float, folder: str) -> Tuple[Optional[float], str]:
    try:
        return _get_local_terrain_database(folder).elevation(lat, lon)
    except Exception as exc:
        return None, f"Local terrain unavailable: {exc}"


def _query_terrain_point(lat: float, lon: float) -> Tuple[Optional[float], str]:
    config = _terrain_settings()
    if config["choice"] in ("local", "local_then_online"):
        value, source = (
            _query_local_terrain_point(lat, lon, config["folder"])
            if config["folder"]
            else (None, "No local terrain folder selected")
        )
        if value is not None or config["choice"] == "local":
            return value, source
    return kmz_query_open_topo_data_elevation_online(lat, lon)


def _query_opentopodata_batch(points: List[Tuple[float, float]], status_callback: Optional[Any] = None) -> Tuple[List[Optional[float]], str]:
    if not points:
        return [], "No terrain locations"
    url = "https://api.opentopodata.org/v1/srtm30m,aster30m,srtm90m"
    locations = "|".join(f"{lat:.7f},{lon:.7f}" for lat, lon in points)
    payload = json.dumps({"locations": locations, "interpolation": "bilinear"}).encode("utf-8")
    request = urllib.request.Request(url, data=payload, headers={"User-Agent": "Flight-Map-Tools-Dashware/31", "Content-Type": "application/json", "Accept": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(request, timeout=30.0) as response:
            data = json.loads(response.read().decode("utf-8", errors="replace"))
        if data.get("status") != "OK":
            return [None] * len(points), str(data.get("error") or data.get("status") or "Terrain lookup failed")
        results = data.get("results") or []
        elevations = [float(item["elevation"]) if item.get("elevation") is not None else None for item in results]
        if len(elevations) < len(points):
            elevations.extend([None] * (len(points) - len(elevations)))
        datasets = sorted({str(item.get("dataset")) for item in results if item.get("dataset")})
        return elevations[:len(points)], "OpenTopoData " + ", ".join(datasets or ["terrain dataset"])
    except Exception as exc:
        if status_callback:
            status_callback(f"Terrain lookup request failed: {exc}")
        return [None] * len(points), str(exc)


def _terrain_elevation_for_records_online(records: List[Dict[str, Any]], status_callback: Optional[Any] = None) -> Tuple[List[Optional[float]], str]:
    """OpenTopoData mode. It samples the route to stay within a public API's limits."""
    valid=[i for i,r in enumerate(records) if r.get('gps') is not None]; output=[None]*len(records)
    if not valid: return output,'No GPS coordinates'
    selected=[valid[0]]; last=valid[0]
    for i in valid[1:-1]:
        if haversine_m(records[last]['gps'],records[i]['gps'])>=DASHWARE_TERRAIN_SAMPLE_DISTANCE_M or float(records[i]['elapsed_s'])-float(records[last]['elapsed_s'])>=DASHWARE_TERRAIN_SAMPLE_TIME_S:
            selected.append(i); last=i
    if selected[-1]!=valid[-1]: selected.append(valid[-1])
    if len(selected)>DASHWARE_TERRAIN_MAX_SAMPLES:
        step=max(1,math.ceil(len(selected)/DASHWARE_TERRAIN_MAX_SAMPLES)); selected=selected[::step]
        if selected[-1]!=valid[-1]: selected.append(valid[-1])
    if status_callback: status_callback(f'Online terrain lookup: querying {len(selected)} path samples and interpolating to {len(records)} CSV rows.')
    elevations=[]; source=''
    for n in range(0,len(selected),DASHWARE_TERRAIN_BATCH_SIZE):
        ids=selected[n:n+DASHWARE_TERRAIN_BATCH_SIZE]; vals,s=_query_opentopodata_batch([records[i]['gps'] for i in ids],status_callback=status_callback)
        elevations.extend(vals)
        if s and not source: source=s
    known=[(i,e) for i,e in zip(selected,elevations) if e is not None]
    if not known: return output,source or 'Online terrain lookup returned no elevations'
    for i,e in known: output[i]=e
    for (a,ae),(b,be) in zip(known,known[1:]):
        at=float(records[a]['elapsed_s']); bt=float(records[b]['elapsed_s'])
        for i in range(a,b+1):
            if records[i].get('gps') is None: continue
            f=(float(records[i]['elapsed_s'])-at)/(bt-at) if bt>at else 0
            output[i]=float(ae)+(float(be)-float(ae))*max(0,min(1,f))
    for i in range(0,known[0][0]+1):
        if records[i].get('gps') is not None: output[i]=known[0][1]
    for i in range(known[-1][0],len(records)):
        if records[i].get('gps') is not None: output[i]=known[-1][1]
    return output,source or 'OpenTopoData'

def _terrain_elevation_for_records_local(records: List[Dict[str, Any]],folder: str,status_callback: Optional[Any]=None) -> Tuple[List[Optional[float]],str]:
    output=[None]*len(records); valid=[i for i,r in enumerate(records) if r.get('gps') is not None]
    if not valid: return output,'No GPS coordinates'
    if not folder: return output,'No local terrain folder selected'
    if status_callback and _terrain_cache_key(folder) not in _TERRAIN_DB_CACHE:
        status_callback('Local terrain first use: recursively opening and indexing every readable DAT/HGT file for this app session...')
    try: db=_get_local_terrain_database(folder)
    except Exception as exc: return output,f'Local terrain unavailable: {exc}'
    if status_callback:
        status_callback(db.use_message())
        status_callback(f'Local terrain lookup: querying every distinct logged GPS coordinate ({len(valid)} GPS row(s)); highest-resolution matching tile is preferred automatically.')
    cache={}; used={}; missing=0
    for i in valid:
        lat,lon=records[i]['gps']; key=(round(float(lat),7),round(float(lon),7))
        if key not in cache: cache[key]=db.elevation(float(lat),float(lon))
        v,s=cache[key]; output[i]=v
        if v is None: missing+=1
        else: used[s]=used.get(s,0)+1
    if status_callback: status_callback(f'Local terrain lookup complete: {len(cache)} distinct coordinate(s), {len(valid)-missing}/{len(valid)} GPS row(s) covered; no flight-path sample interpolation.')
    names=sorted(used,key=lambda x:(-used[x],x)); summary='; '.join(names[:4])
    if len(names)>4: summary+=f'; +{len(names)-4} more tile(s)'
    return output,summary or f'Local terrain folder: {folder}'

def _terrain_elevation_for_records(records: List[Dict[str, Any]], status_callback: Optional[Any] = None) -> Tuple[List[Optional[float]], str]:
    cfg=_terrain_settings()
    if cfg['choice'] in ('local','local_then_online'):
        values,source=_terrain_elevation_for_records_local(records,cfg['folder'],status_callback)
        if any(v is not None for v in values):
            if cfg['choice']=='local_then_online' and any(r.get('gps') is not None and values[i] is None for i,r in enumerate(records)):
                online,osource=_terrain_elevation_for_records_online(records,status_callback); filled=0
                for i,v in enumerate(values):
                    if v is None and online[i] is not None: values[i]=online[i]; filled+=1
                return values,f'{source}; online fallback filled {filled} row(s) from {osource}'
            return values,source
        if cfg['choice']=='local': return values,source
        if status_callback: status_callback(f'Local terrain unavailable; trying online fallback. {source}')
    return _terrain_elevation_for_records_online(records,status_callback)


def _resolve_dashware_takeoff_msl(data: Dict[str, Any], terrain_values: Optional[List[Optional[float]]] = None, status_callback: Optional[Any] = None) -> Tuple[Optional[float], str]:
    """Resolve Dashware takeoff MSL only from terrain at the first relative-zero GPS point.

    The original CSV's initial altitude is deliberately not used here. Betaflight logs can
    briefly contain a takeoff-elevation value before the relative altitude resets to zero;
    Dashware MSL/AGL enrichment instead assumes the aircraft is on the terrain at the first
    valid zero-relative sample.
    """
    records = data["records"]
    candidates = [
        r for r in records
        if r.get("gps") is not None and r.get("relative_alt") is not None
        and abs(float(r.get("relative_alt") or 0.0)) <= ALT_RELATIVE_ZERO_THRESHOLD_M
    ]
    reference = candidates[0] if candidates else next((r for r in records if r.get("gps") is not None), None)
    if reference is None:
        return None, "No valid GPS point for terrain takeoff reference"
    idx = int(reference["row_index"])
    if terrain_values is not None and 0 <= idx < len(terrain_values) and terrain_values[idx] is not None:
        return float(terrain_values[idx]), "selected terrain source at first relative-zero GPS point"
    elev, source = kmz_query_open_topo_data_elevation(float(reference["lat"]), float(reference["lon"]))
    if elev is not None:
        return float(elev), f"{source} at first relative-zero GPS point"
    if status_callback:
        status_callback(f"Takeoff terrain lookup failed: {source}")
    return None, "Terrain takeoff lookup unavailable; CSV altitude was not substituted"


def _attitude_unit_from_header_and_values(header_name: str, values: List[float]) -> str:
    """Detect radians/degrees from the column title first, then from value behaviour."""
    header_lower = str(header_name or "").lower()
    if "rad" in header_lower:
        return "rad"
    if "deg" in header_lower or "°" in str(header_name or ""):
        return "deg"
    clean = [abs(float(v)) for v in values if v is not None and math.isfinite(float(v))]
    if not clean:
        return "unknown"
    p95 = _percentile(clean, 95.0) or 0.0
    return "rad" if p95 <= 7.0 else "deg"


def _compute_attitude_fields(records: List[Dict[str, Any]], headers: Dict[str, str]) -> None:
    """Convert attitude angles to degrees and derive roll/pitch/yaw angular rates."""
    for axis in ("roll", "pitch", "yaw"):
        raw_key = f"{axis}_raw"
        values = [float(r[raw_key]) for r in records if r.get(raw_key) is not None]
        unit = _attitude_unit_from_header_and_values(headers.get(axis, ""), values)
        deg_key = f"{axis}_deg"
        rate_key = f"{axis}_rate_deg_s"
        for record in records:
            raw = record.get(raw_key)
            if raw is None:
                record[deg_key] = None
            elif unit == "rad":
                record[deg_key] = math.degrees(float(raw))
            else:
                record[deg_key] = float(raw)
            record[f"{axis}_source_unit"] = unit

        previous: Optional[int] = None
        for i, record in enumerate(records):
            value = record.get(deg_key)
            if value is None:
                record[rate_key] = None
                continue
            if previous is None:
                record[rate_key] = None
                previous = i
                continue
            dt = float(record.get("elapsed_s", 0.0)) - float(records[previous].get("elapsed_s", 0.0))
            if dt <= 0 or dt > 10:
                record[rate_key] = None
            else:
                delta = _angle_difference_deg(float(value), float(records[previous][deg_key]))
                record[rate_key] = delta / dt
            previous = i


def _choose_dashware_heading_source(records: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Choose original Hdg or centred 2-second GPS course using the user's 3% rule."""
    moving = [
        i for i, r in enumerate(records)
        if r.get("coord_speed_kmh") is not None and float(r.get("coord_speed_kmh")) >= DASHWARE_HEADING_MIN_SPEED_KMH
    ]
    if not moving:
        moving = list(range(len(records)))
    original_coverage = (sum(records[i].get("heading_raw") is not None for i in moving) / len(moving) * 100.0) if moving else 0.0
    gps_coverage = (sum(records[i].get("heading_true_deg") is not None for i in moving) / len(moving) * 100.0) if moving else 0.0
    comparisons: List[float] = []
    for i in moving:
        original = records[i].get("heading_raw")
        gps_course = records[i].get("heading_true_deg")
        if original is None or gps_course is None:
            continue
        comparisons.append(abs(_angle_difference_deg(float(gps_course), float(original))))
    within_pct = 0.0
    bad_pct = 100.0
    if comparisons:
        within = sum(d <= DASHWARE_HEADING_COMPARE_TOLERANCE_DEG for d in comparisons)
        within_pct = within / len(comparisons) * 100.0
        bad_pct = 100.0 - within_pct

    coverage_required = 100.0 - DASHWARE_HEADING_MAX_BAD_PCT
    if original_coverage >= coverage_required and comparisons and bad_pct <= DASHWARE_HEADING_MAX_BAD_PCT:
        source = "original"
        reason = (
            f"original Hdg selected: {original_coverage:.1f}% moving-row coverage and "
            f"{within_pct:.1f}% of comparisons were within {DASHWARE_HEADING_COMPARE_TOLERANCE_DEG:.0f}° of the 2-second GPS course"
        )
    elif gps_coverage >= coverage_required:
        source = "gps"
        reason = (
            f"2-second GPS course selected: {gps_coverage:.1f}% moving-row coverage; "
            f"original/GPS agreement within {DASHWARE_HEADING_COMPARE_TOLERANCE_DEG:.0f}° was {within_pct:.1f}%"
        )
    elif original_coverage >= gps_coverage:
        source = "original"
        reason = f"original Hdg selected as the higher-coverage source ({original_coverage:.1f}% vs {gps_coverage:.1f}%)"
    else:
        source = "gps"
        reason = f"2-second GPS course selected as the higher-coverage source ({gps_coverage:.1f}% vs {original_coverage:.1f}%)"

    for record in records:
        original = record.get("heading_raw")
        gps_course = record.get("heading_true_deg")
        if source == "original":
            selected = original if original is not None else gps_course
        else:
            selected = gps_course if gps_course is not None else original
        record["heading_selected_deg"] = selected
        record["heading_selected_cardinal"] = _cardinal_from_heading(selected)

    previous: Optional[int] = None
    for i, record in enumerate(records):
        value = record.get("heading_selected_deg")
        if value is None:
            record["turn_rate_selected_deg_s"] = None
            continue
        if previous is None:
            record["turn_rate_selected_deg_s"] = None
            previous = i
            continue
        dt = float(record.get("elapsed_s", 0.0)) - float(records[previous].get("elapsed_s", 0.0))
        if dt <= 0 or dt > 10:
            record["turn_rate_selected_deg_s"] = None
        else:
            record["turn_rate_selected_deg_s"] = _angle_difference_deg(float(value), float(records[previous]["heading_selected_deg"])) / dt
        previous = i

    return {
        "source": source,
        "reason": reason,
        "moving_rows": len(moving),
        "comparison_rows": len(comparisons),
        "original_coverage_pct": original_coverage,
        "gps_coverage_pct": gps_coverage,
        "within_tolerance_pct": within_pct,
        "bad_pct": bad_pct,
        "median_diff": _median(comparisons),
        "max_diff": max(comparisons) if comparisons else None,
    }


def _dashware_heading_redundancy(records: List[Dict[str, Any]], tolerance_deg: float = 5.0) -> Dict[str, Any]:
    differences: List[float] = []
    for record in records:
        original = record.get("heading_raw")
        computed = record.get("heading_true_deg")
        speed = record.get("coord_speed_kmh")
        if original is None or computed is None or speed is None or float(speed) < 5.0:
            continue
        differences.append(abs(_angle_difference_deg(float(computed), float(original))))
    if not differences:
        return {"count": 0, "redundant": False, "max_diff": None, "median_diff": None, "within_pct": 0.0}
    within = sum(1 for d in differences if d <= tolerance_deg)
    return {
        "count": len(differences),
        "redundant": len(differences) >= 10 and max(differences) <= tolerance_deg,
        "max_diff": max(differences),
        "median_diff": _median(differences),
        "within_pct": within / len(differences) * 100.0,
    }


def _resolved_dashware_units(settings: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    merged = dict(DEFAULT_PARAMETER_SETTINGS)
    merged.update(settings or {})
    system = str(merged.get("unit_system", "Metric"))
    if system == "Imperial":
        merged.update({
            "distance_unit": "ft", "long_distance_unit": "mi", "speed_unit": "mph",
            "altitude_unit": "ft", "vertical_speed_unit": "ft/s",
            "acceleration_unit": "ft/s²", "efficiency_distance_unit": "mi",
        })
    elif system == "Metric":
        merged.update({
            "distance_unit": "m", "long_distance_unit": "km", "speed_unit": "km/h",
            "altitude_unit": "m", "vertical_speed_unit": "m/s",
            "acceleration_unit": "m/s²", "efficiency_distance_unit": "km",
        })
    for key, default in DEFAULT_PARAMETER_SETTINGS.items():
        if key.endswith("_decimals"):
            try:
                merged[key] = max(0, min(8, int(merged.get(key, default))))
            except Exception:
                merged[key] = default
    if str(merged.get("angular_rate_unit", "deg/s")) not in ("deg/s", "rad/s"):
        merged["angular_rate_unit"] = "deg/s"
    if str(merged.get("elapsed_format", "Seconds")) not in ("Seconds", "Decimal minutes", "Clock H:MM:SS.mmm"):
        merged["elapsed_format"] = "Seconds"
    return merged


def _dashware_unit_slug(unit: str) -> str:
    return {
        "km/h": "kmh", "mph": "mph", "m/s": "mps", "kn": "knots",
        "m": "m", "km": "km", "ft": "ft", "mi": "mi",
        "ft/s": "ftps", "ft/min": "ftmin", "m/s²": "mps2", "ft/s²": "ftps2",
        "deg/s": "degps", "rad/s": "radps",
    }.get(unit, unit.replace("/", "_").replace("²", "2").replace("°", "deg").replace(" ", "_"))


def _convert_distance_m(value: Optional[float], unit: str) -> Optional[float]:
    if value is None: return None
    v = float(value)
    return v if unit == "m" else v / 1000.0 if unit == "km" else v * 3.280839895 if unit == "ft" else v / 1609.344 if unit == "mi" else v


def _convert_distance_km(value: Optional[float], unit: str) -> Optional[float]:
    return _convert_distance_m(None if value is None else float(value) * 1000.0, unit)


def _convert_speed_kmh(value: Optional[float], unit: str) -> Optional[float]:
    if value is None: return None
    v = float(value)
    return v if unit == "km/h" else v * 0.621371192 if unit == "mph" else v / 3.6 if unit == "m/s" else v * 0.539956803 if unit == "kn" else v


def _convert_altitude_m(value: Optional[float], unit: str, cap: bool = False) -> Optional[float]:
    if value is None: return None
    v = float(value)
    if cap and v > 121.92:
        v = 121.92
    return v if unit == "m" else v * 3.280839895 if unit == "ft" else v / 1000.0 if unit == "km" else v / 1609.344 if unit == "mi" else v


def _convert_vertical_speed(value: Optional[float], unit: str) -> Optional[float]:
    if value is None: return None
    v = float(value)
    return v if unit == "m/s" else v * 3.280839895 if unit == "ft/s" else v * 196.8503937 if unit == "ft/min" else v


def _convert_acceleration(value: Optional[float], unit: str) -> Optional[float]:
    if value is None: return None
    v = float(value)
    return v if unit == "m/s²" else v * 3.280839895 if unit == "ft/s²" else v


def _convert_angular_rate_deg_s(value: Optional[float], unit: str) -> Optional[float]:
    if value is None:
        return None
    v = float(value)
    return math.radians(v) if unit == "rad/s" else v


def _format_elapsed_value(seconds: Optional[float], elapsed_format: str) -> Any:
    if seconds is None: return None
    value = max(0.0, float(seconds))
    if elapsed_format == "Decimal minutes":
        return value / 60.0
    if elapsed_format == "Clock H:MM:SS.mmm":
        hours = int(value // 3600); minutes = int((value % 3600) // 60); secs = value % 60
        return f"{hours}:{minutes:02d}:{secs:06.3f}"
    return value


def _dashware_column_name(field_id: str, settings: Dict[str, Any]) -> str:
    units = _resolved_dashware_units(settings)
    elapsed_format = str(units.get("elapsed_format", "Seconds"))
    if field_id == "elapsed":
        return "Elapsed_Time_s" if elapsed_format == "Seconds" else "Elapsed_Time_min" if elapsed_format == "Decimal minutes" else "Elapsed_Time_HHMMSS"
    fixed = {
        "latitude": "Latitude_deg", "longitude": "Longitude_deg",
        "heading_deg": "Heading_True_deg", "heading_cardinal": "Heading_Cardinal",
        "rssi": "RSSI_Best_dBm", "power": "Power_W",
        "energy_used": "Energy_Used_Wh", "throttle": "Throttle_pct",
    }
    if field_id in fixed: return fixed[field_id]
    if field_id == "coord_speed": return f"Coordinate_Speed_{_dashware_unit_slug(units['speed_unit'])}"
    if field_id == "distance_home": return f"Distance_From_Home_{_dashware_unit_slug(units['distance_unit'])}"
    if field_id == "cumulative_distance": return f"Cumulative_Distance_{_dashware_unit_slug(units['long_distance_unit'])}"
    if field_id == "altitude_msl": return f"Altitude_MSL_{_dashware_unit_slug(units['altitude_unit'])}"
    if field_id == "terrain_msl": return f"Terrain_Elevation_{_dashware_unit_slug(units['altitude_unit'])}"
    if field_id == "altitude_agl": return f"Altitude_AGL_{_dashware_unit_slug(units['altitude_unit'])}"
    if field_id == "vertical_speed": return f"Vertical_Speed_{_dashware_unit_slug(units['vertical_speed_unit'])}"
    if field_id == "logged_vspd": return f"Logged_VSpd_{_dashware_unit_slug(units['vertical_speed_unit'])}"
    if field_id == "temperature": return "Temperature_C"
    if field_id == "acceleration": return f"Acceleration_{_dashware_unit_slug(units['acceleration_unit'])}"
    if field_id == "turn_rate": return f"Ground_Track_Turn_Rate_{_dashware_unit_slug(units['angular_rate_unit'])}"
    if field_id == "roll_rate": return f"Roll_Rate_{_dashware_unit_slug(units['angular_rate_unit'])}"
    if field_id == "pitch_rate": return f"Pitch_Rate_{_dashware_unit_slug(units['angular_rate_unit'])}"
    if field_id == "yaw_rate": return f"Yaw_Rate_{_dashware_unit_slug(units['angular_rate_unit'])}"
    if field_id == "efficiency_mah": return f"Efficiency_mAh_per_{units['efficiency_distance_unit']}"
    if field_id == "efficiency_wh": return f"Efficiency_Wh_per_{units['efficiency_distance_unit']}"
    return next((f["column"] for f in DASHWARE_FIELDS if f["id"] == field_id), field_id)


def _dashware_value(record: Dict[str, Any], field_id: str, takeoff_msl: Optional[float], terrain_value: Optional[float], settings: Dict[str, Any]) -> Any:
    units = _resolved_dashware_units(settings)
    relative = record.get("relative_alt")
    altitude_msl = takeoff_msl + float(relative) if takeoff_msl is not None and relative is not None else None
    cap = bool(units.get("joke_altitude_cap", False))
    if field_id == "elapsed": return _format_elapsed_value(record.get("elapsed_s"), str(units.get("elapsed_format", "Seconds")))
    if field_id == "latitude": return record.get("lat")
    if field_id == "longitude": return record.get("lon")
    if field_id == "heading_deg": return record.get("heading_selected_deg")
    if field_id == "heading_cardinal": return record.get("heading_selected_cardinal", "")
    if field_id == "coord_speed": return _convert_speed_kmh(record.get("coord_speed_kmh"), units["speed_unit"])
    if field_id == "distance_home": return _convert_distance_m(record.get("distance_home_m"), units["distance_unit"])
    if field_id == "cumulative_distance": return _convert_distance_km(record.get("cumulative_distance_km"), units["long_distance_unit"])
    if field_id == "altitude_msl": return _convert_altitude_m(altitude_msl, units["altitude_unit"], cap)
    if field_id == "terrain_msl": return _convert_altitude_m(terrain_value, units["altitude_unit"], cap)
    if field_id == "altitude_agl":
        agl = altitude_msl - terrain_value if altitude_msl is not None and terrain_value is not None else None
        if agl is not None and bool(units.get("clamp_negative_agl", True)):
            agl = max(0.0, float(agl))
        return _convert_altitude_m(agl, units["altitude_unit"], cap)
    if field_id == "vertical_speed": return _convert_vertical_speed(record.get("vertical_speed_mps"), units["vertical_speed_unit"])
    if field_id == "logged_vspd": return _convert_vertical_speed(record.get("vspd_logged_mps"), units["vertical_speed_unit"])
    if field_id == "temperature": return record.get("temperature_c")
    if field_id == "acceleration": return _convert_acceleration(record.get("acceleration_mps2"), units["acceleration_unit"])
    if field_id == "turn_rate": return _convert_angular_rate_deg_s(record.get("turn_rate_selected_deg_s"), units["angular_rate_unit"])
    if field_id == "roll_rate": return _convert_angular_rate_deg_s(record.get("roll_rate_deg_s"), units["angular_rate_unit"])
    if field_id == "pitch_rate": return _convert_angular_rate_deg_s(record.get("pitch_rate_deg_s"), units["angular_rate_unit"])
    if field_id == "yaw_rate": return _convert_angular_rate_deg_s(record.get("yaw_rate_deg_s"), units["angular_rate_unit"])
    if field_id == "rssi": return record.get("rssi")
    if field_id == "power": return record.get("power_w")
    if field_id == "energy_used": return record.get("energy_used_Wh")
    if field_id == "efficiency_mah":
        value = record.get("efficiency_mAh_km")
        return None if value is None else float(value) * 1.609344 if units["efficiency_distance_unit"] == "mi" else value
    if field_id == "efficiency_wh":
        value = record.get("efficiency_Wh_km")
        return None if value is None else float(value) * 1.609344 if units["efficiency_distance_unit"] == "mi" else value
    if field_id == "throttle":
        value = record.get("throttle_pct")
        return None if value is None else int(round(float(value)))
    return None


def _dashware_decimals(field_id: str, settings: Dict[str, Any]) -> int:
    units = _resolved_dashware_units(settings)
    if field_id in ("latitude", "longitude"):
        return 7
    if field_id == "elapsed":
        return 2 if str(units.get("elapsed_format", "Seconds")) == "Decimal minutes" else 3
    if field_id in ("coord_speed", "vertical_speed", "logged_vspd"):
        return int(units["speed_decimals"])
    if field_id == "temperature":
        return 1
    if field_id == "distance_home":
        return int(units["long_decimals"] if units["distance_unit"] in ("km", "mi") else units["short_decimals"])
    if field_id == "cumulative_distance":
        return int(units["long_decimals"] if units["long_distance_unit"] in ("km", "mi") else units["short_decimals"])
    if field_id in ("altitude_msl", "terrain_msl", "altitude_agl"):
        return int(units["altitude_decimals"])
    if field_id == "throttle":
        return 0
    return int(units["general_decimals"])


def _write_gpx_from_records(csv_path: str, records: List[Dict[str, Any]], takeoff_msl: Optional[float], altitude_source: str = "") -> str:
    out_path = os.path.splitext(csv_path)[0] + " (track).gpx"
    creator = f"Flight Map Tools {str(APP_VERSION_NUMBER).lstrip('vV')}"
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<gpx version="1.1" creator="{escape(creator)}" xmlns="http://www.topografix.com/GPX/1/1" xmlns:fpv="https://joshs-air.example/fpv">',
        f'<metadata><name>{escape(os.path.basename(csv_path))}</name></metadata>',
        '<trk><name>FPV flight track</name>',
    ]
    in_segment = False
    for record in records:
        valid = record.get("gps") is not None and (record.get("sats") is None or float(record.get("sats")) >= ANALYSIS_MIN_TRACK_SATS)
        if not valid:
            if in_segment:
                lines.append('</trkseg>')
                in_segment = False
            continue
        if not in_segment:
            lines.append('<trkseg>')
            in_segment = True
        lat = float(record["lat"]); lon = float(record["lon"]); relative = record.get("relative_alt")
        if altitude_source in ("ardupilot_asl", "inav_asl", "asl_unknown") and record.get("alt_raw") is not None:
            ele = float(record["alt_raw"])
        else:
            ele = takeoff_msl + float(relative) if takeoff_msl is not None and relative is not None else None
        lines.append(f'<trkpt lat="{lat:.7f}" lon="{lon:.7f}">')
        if ele is not None:
            lines.append(f'<ele>{float(ele):.2f}</ele>')

        # Newer ELRS/CRSF logs can include a GPS UTC date+time sensor.  GPX gets that UTC
        # timestamp when available; the app's flight timing and displayed times still use
        # the separate local EdgeTX Date + Time columns.
        utc_dt = record.get("utc_datetime")
        if isinstance(utc_dt, datetime):
            iso = utc_dt.isoformat()
            if iso.endswith("+00:00"):
                iso = iso[:-6] + "Z"
            elif utc_dt.tzinfo is None:
                iso += "Z"
            lines.append(f'<time>{escape(iso)}</time>')
        elif record.get("date") and record.get("time"):
            try:
                iso = datetime.fromisoformat(f"{record['date']} {record['time']}").isoformat()
                lines.append(f'<time>{escape(iso)}</time>')
            except Exception:
                pass
        if record.get("sats") is not None:
            lines.append(f'<sat>{int(round(float(record["sats"])))}</sat>')
        lines.append('<extensions>')
        if record.get("gspd") is not None:
            lines.append(f'<fpv:speed_kmh>{float(record["gspd"]):.3f}</fpv:speed_kmh>')
        if record.get("alt_raw") is not None:
            lines.append(f'<fpv:raw_altitude_m>{float(record["alt_raw"]):.3f}</fpv:raw_altitude_m>')
        if relative is not None:
            lines.append(f'<fpv:relative_altitude_m>{float(relative):.3f}</fpv:relative_altitude_m>')
        if record.get("vspd_logged_mps") is not None:
            lines.append(f'<fpv:logged_vspd_mps>{float(record["vspd_logged_mps"]):.3f}</fpv:logged_vspd_mps>')
        if record.get("temperature_c") is not None:
            lines.append(f'<fpv:temperature_c>{float(record["temperature_c"]):.2f}</fpv:temperature_c>')
        if record.get("flight_mode"):
            lines.append(f'<fpv:flight_mode>{escape(str(record["flight_mode"]))}</fpv:flight_mode>')
        lines.append('</extensions></trkpt>')
    if in_segment:
        lines.append('</trkseg>')
    lines.append('</trk></gpx>')
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))
    return out_path

def _negative_agl_summary(records: List[Dict[str, Any]], takeoff_msl: Optional[float], terrain_values: List[Optional[float]]) -> Dict[str, Any]:
    negative_rows: List[int] = []
    raw_values: List[Optional[float]] = []
    for i, record in enumerate(records):
        terrain = terrain_values[i] if i < len(terrain_values) else None
        relative = record.get("relative_alt")
        agl = (float(takeoff_msl) + float(relative) - float(terrain)) if takeoff_msl is not None and relative is not None and terrain is not None else None
        raw_values.append(agl)
        if agl is not None and agl < 0:
            negative_rows.append(i)
    if not negative_rows:
        return {"rows": 0, "seconds": 0.0, "episodes": 0, "longest_s": 0.0, "minimum_m": None}
    positive_deltas = []
    for i in range(1, len(records)):
        dt = float(records[i].get("elapsed_s", 0.0)) - float(records[i - 1].get("elapsed_s", 0.0))
        if 0 < dt < 60:
            positive_deltas.append(dt)
    sample_period = _median(positive_deltas) or 0.0
    runs: List[Tuple[int, int, float]] = []
    start: Optional[int] = None
    for i in range(len(records)):
        is_negative = raw_values[i] is not None and float(raw_values[i]) < 0
        if is_negative and start is None:
            start = i
        if start is not None and (not is_negative or i == len(records) - 1):
            end = i - 1 if not is_negative else i
            duration = max(0.0, float(records[end].get("elapsed_s", 0.0)) - float(records[start].get("elapsed_s", 0.0)) + sample_period)
            runs.append((start, end, duration))
            start = None
    return {
        "rows": len(negative_rows),
        "seconds": sum(run[2] for run in runs),
        "episodes": len(runs),
        "longest_s": max((run[2] for run in runs), default=0.0),
        "minimum_m": min(float(raw_values[i]) for i in negative_rows),
    }


def enrich_csv_for_dashware(csv_path: str, selected_field_ids: List[str], create_gpx: bool = False, throttle_col_name: str = "CH3(us)", status_callback: Optional[Any] = None, parameter_settings: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    valid_ids = {f["id"] for f in DASHWARE_FIELDS}
    selected = [field_id for field_id in selected_field_ids if field_id in valid_ids]
    if "elapsed" not in selected:
        selected.insert(0, "elapsed")
    else:
        selected = ["elapsed"] + [x for x in selected if x != "elapsed"]
    if not selected and not create_gpx:
        raise ValueError("Select at least one added column or enable GPX output.")
    settings = _resolved_dashware_units(parameter_settings)
    data = _read_telemetry_records(csv_path, throttle_col_name=throttle_col_name)
    records = data["records"]

    if status_callback:
        stack_info = data.get("flight_stack", {})
        status_callback(
            f"Telemetry profile: {stack_info.get('stack', 'unknown')} "
            f"({stack_info.get('confidence', 'low')} confidence) — {stack_info.get('reason', 'no identifying flight-mode data')}."
        )
        if data.get("datetime_sources", {}).get("utc_datetime_index") is not None:
            dt_sources = data.get("datetime_sources", {})
            observed_offset = dt_sources.get("observed_utc_minus_local_hours")
            offset_note = f" Observed UTC-minus-local offset: {float(observed_offset):.2f} h." if observed_offset is not None else ""
            status_callback("Separate GPS/CRSF UTC date-time sensor detected; local EdgeTX Date + Time remains authoritative for elapsed time and displayed local timestamps." + offset_note)
        autonomy = data.get("autonomy", {})
        if autonomy.get("classification") == "semi_autonomous":
            modes = ", ".join(autonomy.get("autonomous_modes", [])) or "controller-managed mode(s)"
            status_callback(
                f"Semi-autonomous flight detected: {autonomy.get('autonomous_fraction', 0.0) * 100.0:.1f}% of logged time in "
                f"controller-managed throttle/navigation mode(s) ({modes}). RC throttle is an input command, not guaranteed actual motor/TECS output."
            )
        if "logged_vspd" in selected and not any(r.get("vspd_logged_mps") is not None for r in records):
            status_callback("Logged VSpd was selected, but no compatible vertical-speed telemetry column was present; that added column will be blank.")
        if "temperature" in selected and not any(r.get("temperature_c") is not None for r in records):
            status_callback("Temperature was selected, but no compatible Temp telemetry column was present; that added column will be blank.")
        if "pitch_rate" in selected:
            pitch_names = _matching_col_indices(data.get("header", []), ["Ptch(°)", "Pitch(°)", "Ptch(rad)", "Pitch(rad)"], startswith=False)
            if len(pitch_names) > 1:
                chosen = data.get("indices", {}).get("pitch")
                chosen_name = data.get("header", [])[chosen] if isinstance(chosen, int) and chosen < len(data.get("header", [])) else "unknown"
                status_callback(f"Duplicate pitch telemetry headers detected; selected the populated/dynamic candidate at CSV column {int(chosen) + 1 if isinstance(chosen, int) else '?'} ({chosen_name}).")

    heading_check = _choose_dashware_heading_source(records)
    if status_callback and any(x in selected for x in ("heading_deg", "heading_cardinal", "turn_rate")):
        status_callback(f"Heading source: {heading_check['reason']}.")
    if status_callback and any(x in selected for x in ("roll_rate", "pitch_rate", "yaw_rate")):
        detected_parts = []
        for axis in ("roll", "pitch", "yaw"):
            source_unit = next((str(r.get(f"{axis}_source_unit")) for r in records if r.get(f"{axis}_source_unit") not in (None, "unknown")), "unavailable")
            detected_parts.append(f"{axis}={source_unit}")
        status_callback(
            "Attitude angle units detected from headers/data: " + ", ".join(detected_parts)
            + f"; generated angular rates use {settings.get('angular_rate_unit', 'deg/s')}."
        )
    if status_callback:
        status_callback("Elapsed time is calculated from the local EdgeTX Date + Time fields regardless of CSV column order; the median row interval is used only when an individual local timestamp is missing.")

    needs_terrain = any(x in selected for x in ("terrain_msl", "altitude_agl"))
    needs_takeoff = any(x in selected for x in ("altitude_msl", "altitude_agl")) or create_gpx
    terrain_values: List[Optional[float]] = [None] * len(records)
    terrain_source = "Not requested"
    if needs_terrain:
        terrain_values, terrain_source = _terrain_elevation_for_records(records, status_callback=status_callback)
    takeoff_msl: Optional[float] = None
    takeoff_source = "Not requested"
    if needs_takeoff:
        takeoff_msl, takeoff_source = _resolve_dashware_takeoff_msl(data, terrain_values if needs_terrain else None, status_callback=status_callback)
        if status_callback:
            status_callback(f"Takeoff terrain reference: {takeoff_source}" + (f" ({takeoff_msl:.1f} m)" if takeoff_msl is not None else ""))

    agl_summary = _negative_agl_summary(records, takeoff_msl, terrain_values) if "altitude_agl" in selected else {"rows": 0, "seconds": 0.0, "episodes": 0, "longest_s": 0.0, "minimum_m": None}
    if status_callback and agl_summary.get("rows"):
        status_callback(
            f"Alt AGL terrain-model correction: {agl_summary['rows']} row(s) across {agl_summary['episodes']} episode(s), "
            f"{agl_summary['seconds']:.1f} s total, minimum raw AGL {agl_summary['minimum_m']:.1f} m; generated AGL values were clamped to 0."
        )

    extra_columns = [_dashware_column_name(x, settings) for x in selected]
    out_path = os.path.splitext(csv_path)[0] + " (Dashware).csv"
    with open(out_path, "w", encoding="utf-8-sig", newline="") as f:
        writer = csv.writer(f, dialect=data["dialect"])
        writer.writerow(list(data["header"]) + extra_columns)
        for i, record in enumerate(records):
            values: List[str] = []
            for field_id in selected:
                value = _dashware_value(record, field_id, takeoff_msl, terrain_values[i] if i < len(terrain_values) else None, settings)
                if isinstance(value, str): values.append(value)
                elif isinstance(value, int): values.append(str(value))
                else: values.append(_format_dashware_number(value, decimals=_dashware_decimals(field_id, settings)))
            writer.writerow(list(record["row"]) + values)
    gpx_path = _write_gpx_from_records(csv_path, records, takeoff_msl, data.get("altitude_source", "")) if create_gpx else None
    return {
        "csv": out_path, "gpx": gpx_path, "rows": len(records), "columns": extra_columns,
        "terrain_source": terrain_source, "takeoff_source": takeoff_source,
        "takeoff_msl": takeoff_msl, "heading_comparison": heading_check,
        "agl_correction": agl_summary, "flight_stack": data.get("flight_stack", {}),
        "autonomy": data.get("autonomy", {}), "datetime_sources": data.get("datetime_sources", {}),
    }

# ============================================================
# ---------------------------------------------------------------------------
# 3D KMZ export engine (KMZ code v7)
# ---------------------------------------------------------------------------
KMZ_CODE_VERSION_NUMBER = "7"
KMZ_MIN_SATS = MIN_SATS
KMZ_DEDUP_DECIMALS = DEDUP_DECIMALS
KMZ_NEAR_ZERO_ALT_M = 10.0
KMZ_MSL_LOOKING_ALT_M = 80.0
KMZ_ELEVATION_API_TIMEOUT_S = 12.0
KMZ_ELEVATION_DATASETS = ["srtm30m", "aster30m", "srtm90m"]

KMZ_ROYGBIV_KML = [
    "FF0000FF",  # red
    "FF00A5FF",  # orange
    "FF00FFFF",  # yellow
    "FF008000",  # green
    "FFFF0000",  # blue
    "FF82004B",  # indigo
    "FFE22B8A",  # violet
]

@dataclass
class KMZPoint3D:
    lat: float
    lon: float
    alt_msl: float

@dataclass
class KMZSegment:
    mode: str
    points: List[KMZPoint3D]

@dataclass
class KMZFirstSample:
    lat: float
    lon: float
    raw_alt: float

@dataclass
class KMZAltitudeOptions:
    mode: str  # auto, manual, ask_each, csv_only
    manual_takeoff_msl: Optional[float]
    visual_offset_m: float = 0.0
    confirm_online: bool = False
    min_sats: int = MIN_SATS

@dataclass
class KMZTakeoffReference:
    override_takeoff_msl: Optional[float]
    force_relative_from_start: bool
    meta: Dict[str, str]
    input_altitude_type: str = "csv_logic"  # csv_logic, relative, asl
    csv_takeoff_msl: Optional[float] = None


def kmz_escape(text_value: str) -> str:
    return (text_value or "").replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def kmz_safe_offset_suffix(offset_m: float) -> str:
    v = float(offset_m)
    sign = "p" if v >= 0 else "m"
    mag = abs(v)
    if abs(mag - round(mag)) < 1e-9:
        val = str(int(round(mag)))
    else:
        val = (f"{mag:.2f}".rstrip("0").rstrip(".")).replace(".", "p")
    return f"_{sign}{val}m_comp"


def kmz_inspect_first_good_sample(csv_path: str, min_sats: int = MIN_SATS) -> Tuple[Optional[KMZFirstSample], Optional[str]]:
    try:
        with open(csv_path, "r", encoding="utf-8-sig", errors="replace", newline="") as f:
            dialect = sniff_dialect(f)
            reader = csv.reader(f, dialect)
            header = next(reader, None)
            if not header:
                return None, "Empty CSV."
            gps_idx = _find_any_col_index(header, ["GPS", "gps"])
            sats_idx = _find_any_col_index(header, ["sats", "Sats"])
            alt_idx = _find_any_col_index(header, ["Alt(m)", "Alt", "alt (m)", "altm"])
            if gps_idx is None:
                return None, "No GPS column found."
            if sats_idx is None:
                return None, "No sats/Sats column found."
            if alt_idx is None:
                return None, "No Alt(m) column found."
            for row in reader:
                sv = _parse_float(_clean_cell(row, sats_idx))
                if sv is None or sv < float(min_sats):
                    continue
                gps = _parse_gps_cell(_clean_cell(row, gps_idx))
                if gps is None:
                    continue
                raw_alt = _parse_float(_clean_cell(row, alt_idx))
                if raw_alt is None:
                    continue
                return KMZFirstSample(lat=float(gps[0]), lon=float(gps[1]), raw_alt=float(raw_alt)), None
        return None, "No row found with valid GPS, sats, and Alt."
    except Exception as exc:
        return None, f"Could not inspect CSV: {exc}"


def kmz_query_open_topo_data_elevation_online(lat: float, lon: float) -> Tuple[Optional[float], str]:
    locations = urllib.parse.quote(f"{lat:.7f},{lon:.7f}", safe=",")
    errors: List[str] = []
    for dataset in KMZ_ELEVATION_DATASETS:
        url = f"https://api.opentopodata.org/v1/{dataset}?locations={locations}"
        try:
            req = urllib.request.Request(
                url,
                headers={
                    "User-Agent": "Flight-Map-Tools-KMZ-Altitude-Reference/7.0",
                    "Accept": "application/json",
                },
            )
            with urllib.request.urlopen(req, timeout=KMZ_ELEVATION_API_TIMEOUT_S) as response:
                payload = response.read().decode("utf-8", errors="replace")
            data = json.loads(payload)
            if data.get("status") != "OK":
                errors.append(f"{dataset}: status {data.get('status')}")
                continue
            results = data.get("results") or []
            if not results:
                errors.append(f"{dataset}: no results")
                continue
            elevation = results[0].get("elevation")
            if elevation is None:
                errors.append(f"{dataset}: elevation was null")
                continue
            return float(elevation), f"OpenTopoData {dataset}"
        except Exception as exc:
            errors.append(f"{dataset}: {exc}")
    return None, "; ".join(errors) if errors else "Unknown lookup failure"


def kmz_query_open_topo_data_elevation(lat: float, lon: float) -> Tuple[Optional[float], str]:
    """Use the terrain source selected in shared settings."""
    return _query_terrain_point(lat,lon)



def kmz_detect_altitude_source_csv(csv_path: str) -> str:
    """Use the same firmware-aware altitude semantics as maps, analysis and Dashware."""
    return detect_altitude_source_csv(csv_path)


def _kmz_read_csv_altitude_samples(csv_path: str) -> List[Dict[str, Any]]:
    """Read raw Alt(m) samples for KMZ source/fallback decisions."""
    samples: List[Dict[str, Any]] = []
    try:
        with open(csv_path, "r", encoding="utf-8-sig", errors="replace", newline="") as f:
            dialect = sniff_dialect(f)
            reader = csv.reader(f, dialect)
            header = next(reader, None)
            if not header:
                return samples
            alt_idx = _find_any_col_index(header, ["Alt(m)", "Alt", "alt (m)", "altm"])
            if alt_idx is None:
                return samples
            for row_num, row in enumerate(reader):
                raw_alt = _parse_float(_clean_cell(row, alt_idx))
                if raw_alt is not None:
                    samples.append({"alt": float(raw_alt), "row": row_num})
    except Exception:
        pass
    return samples


def _kmz_csv_takeoff_alt_from_betaflight_msl(samples: List[Dict[str, Any]]) -> Optional[float]:
    """
    Return a CSV-provided takeoff elevation/MSL sample when Betaflight briefly logged one.
    This is only used as a fallback when online lookup is unavailable.
    """
    if not samples:
        return None

    raw_values = [float(s.get("alt", 0.0)) for s in samples]
    has_near_zero_after_high = False
    for i, value in enumerate(raw_values):
        if abs(value) >= KMZ_MSL_LOOKING_ALT_M:
            if any(abs(v) <= KMZ_NEAR_ZERO_ALT_M for v in raw_values[i + 1:i + 180]):
                has_near_zero_after_high = True
                break
    if not has_near_zero_after_high:
        return None

    high_values = [v for v in raw_values[:180] if abs(v) >= KMZ_MSL_LOOKING_ALT_M]
    if not high_values:
        return None

    # Use the median-ish middle value so a single bad sample does not become the reference.
    high_values.sort()
    return float(high_values[len(high_values) // 2])

def kmz_resolve_takeoff_reference(csv_path: str, altitude_options: KMZAltitudeOptions, manual_fallback_callback: Optional[Any] = None, confirm_callback: Optional[Any] = None, status_callback: Optional[Any] = None) -> KMZTakeoffReference:
    """
    Resolve the takeoff elevation used for KMZ output.

    v32 behaviour:
      - ASL/MSL logs use their absolute CSV altitude directly unless the user explicitly chooses manual.
      - relative-altitude logs (ArduPilot, Betaflight, INAV, or unknown) use the selected terrain reference by default.
      - If auto lookup fails, the script falls back to CSV-provided Betaflight MSL/elevation
        if present; otherwise it asks for manual elevation.
      - There is no online-elevation confirmation checkbox anymore.
    """
    def status(msg: str) -> None:
        if status_callback:
            try:
                status_callback(msg)
            except Exception:
                pass

    meta: Dict[str, str] = {}
    min_sats = int(getattr(altitude_options, "min_sats", MIN_SATS) or MIN_SATS)
    sample, err = kmz_inspect_first_good_sample(csv_path, min_sats=min_sats)
    if sample is None:
        meta["takeoff_reference_source"] = "unavailable"
        if err:
            meta["takeoff_reference_note"] = err
        return KMZTakeoffReference(override_takeoff_msl=None, force_relative_from_start=False, meta=meta, input_altitude_type="csv_logic")

    meta["first_good_gps"] = f"{sample.lat:.7f},{sample.lon:.7f}"

    alt_source = kmz_detect_altitude_source_csv(csv_path)
    raw_alt_samples = _kmz_read_csv_altitude_samples(csv_path)
    csv_bf_takeoff = _kmz_csv_takeoff_alt_from_betaflight_msl(raw_alt_samples)

    # Manual means "use this as the takeoff elevation".
    # For ASL logs, keep the CSV's relative shape by subtracting CSV takeoff and adding manual.
    if altitude_options.mode == "manual":
        elev = float(altitude_options.manual_takeoff_msl or 0.0)
        meta["takeoff_reference_source"] = f"manual ({elev:.2f} m)"
        meta["used_takeoff_alt_m"] = f"{elev:.2f}"
        if _altitude_source_is_asl(alt_source):
            csv_takeoff = sample.raw_alt
            meta["csv_takeoff_alt_m"] = f"{csv_takeoff:.2f}"
            meta["takeoff_reference_note"] = "Manual takeoff elevation applied to ASL CSV by shifting the whole path."
            return KMZTakeoffReference(override_takeoff_msl=elev, force_relative_from_start=False, meta=meta, input_altitude_type="asl", csv_takeoff_msl=csv_takeoff)
        return KMZTakeoffReference(override_takeoff_msl=elev, force_relative_from_start=True, meta=meta, input_altitude_type="relative", csv_takeoff_msl=None)

    if altitude_options.mode == "csv_only":
        if _altitude_source_is_asl(alt_source):
            meta["takeoff_reference_source"] = f"CSV ASL/MSL ({sample.raw_alt:.2f} m)"
            meta["used_takeoff_alt_m"] = f"{sample.raw_alt:.2f}"
            return KMZTakeoffReference(override_takeoff_msl=None, force_relative_from_start=False, meta=meta, input_altitude_type="asl", csv_takeoff_msl=sample.raw_alt)
        if csv_bf_takeoff is not None:
            meta["takeoff_reference_source"] = f"CSV Betaflight initial MSL ({csv_bf_takeoff:.2f} m)"
            meta["used_takeoff_alt_m"] = f"{csv_bf_takeoff:.2f}"
        else:
            meta["takeoff_reference_source"] = "CSV altitude only; no takeoff elevation reference found"
        return KMZTakeoffReference(override_takeoff_msl=None, force_relative_from_start=False, meta=meta, input_altitude_type="csv_logic", csv_takeoff_msl=csv_bf_takeoff)

    # Auto mode: genuine ASL/MSL input already contains absolute altitude, so do not lookup.
    if _altitude_source_is_asl(alt_source):
        meta["takeoff_reference_source"] = f"CSV ASL/MSL ({sample.raw_alt:.2f} m)"
        meta["used_takeoff_alt_m"] = f"{sample.raw_alt:.2f}"
        return KMZTakeoffReference(override_takeoff_msl=None, force_relative_from_start=False, meta=meta, input_altitude_type="asl", csv_takeoff_msl=sample.raw_alt)

    # Auto mode for relative-altitude inputs: resolve a terrain takeoff reference.
    found_elev: Optional[float] = None
    found_source = ""
    status(f"Looking up takeoff elevation for {os.path.basename(csv_path)}...")
    found_elev, found_source = kmz_query_open_topo_data_elevation(sample.lat, sample.lon)

    if found_elev is not None:
        meta["takeoff_reference_source"] = f"{found_source} ({found_elev:.2f} m)"
        meta["used_takeoff_alt_m"] = f"{found_elev:.2f}"
        return KMZTakeoffReference(override_takeoff_msl=float(found_elev), force_relative_from_start=True, meta=meta, input_altitude_type="relative", csv_takeoff_msl=None)

    status(f"Online elevation lookup failed for {os.path.basename(csv_path)}. {found_source}")
    if csv_bf_takeoff is not None:
        meta["takeoff_reference_source"] = f"CSV Betaflight initial MSL fallback ({csv_bf_takeoff:.2f} m)"
        meta["takeoff_lookup_failure"] = found_source
        meta["used_takeoff_alt_m"] = f"{csv_bf_takeoff:.2f}"
        return KMZTakeoffReference(override_takeoff_msl=float(csv_bf_takeoff), force_relative_from_start=True, meta=meta, input_altitude_type="relative", csv_takeoff_msl=csv_bf_takeoff)

    if manual_fallback_callback is not None:
        elev = manual_fallback_callback(csv_path, sample, found_source)
        if elev is None:
            raise RuntimeError("KMZ export cancelled because no takeoff elevation was provided.")
        meta["takeoff_reference_source"] = f"manual after lookup failed ({float(elev):.2f} m)"
        meta["takeoff_lookup_failure"] = found_source
        meta["used_takeoff_alt_m"] = f"{float(elev):.2f}"
        return KMZTakeoffReference(override_takeoff_msl=float(elev), force_relative_from_start=True, meta=meta, input_altitude_type="relative", csv_takeoff_msl=None)

    meta["takeoff_reference_source"] = "CSV altitude only after lookup failed; no takeoff elevation reference found"
    meta["takeoff_lookup_failure"] = found_source
    return KMZTakeoffReference(override_takeoff_msl=None, force_relative_from_start=False, meta=meta, input_altitude_type="csv_logic", csv_takeoff_msl=None)


def kmz_read_segments_from_csv(csv_path: str, takeoff_reference: Optional[KMZTakeoffReference] = None, visual_offset_m: float = 0.0, min_sats: int = MIN_SATS) -> Tuple[List[KMZSegment], Dict[str, int], Dict[str, str]]:
    """
    Read CSV into 3D KML segments.

    Altitude conversion is based on the resolved takeoff reference:
      - input_altitude_type='asl': raw Alt(m) is ASL and is used directly, or shifted
        to a manual takeoff elevation if one was supplied.
      - input_altitude_type='relative': raw Alt(m) is Betaflight relative altitude;
        startup/later MSL glitches are skipped before adding the used takeoff altitude.
      - input_altitude_type='csv_logic': offline fallback using CSV MSL-then-relative logic.
    """
    stats = {"rows": 0, "kept": 0, "deduped": 0, "missing_gps": 0, "low_sats": 0, "missing_alt": 0, "segments": 0, "four_sat_rows_total": 0, "four_sat_rows_kept": 0}
    meta: Dict[str, str] = {}
    if takeoff_reference is not None:
        meta.update(takeoff_reference.meta)

    segments: List[KMZSegment] = []
    current_mode = ""
    current_points: List[KMZPoint3D] = []
    last_key: Optional[Tuple[float, float]] = None

    with open(csv_path, "r", encoding="utf-8-sig", errors="replace", newline="") as f:
        dialect = sniff_dialect(f)
        reader = csv.reader(f, dialect)
        header = next(reader, None)
        if not header:
            meta["error"] = "Empty CSV."
            return [], stats, meta
        gps_idx = _find_any_col_index(header, ["GPS", "gps"])
        sats_idx = _find_any_col_index(header, ["sats", "Sats"])
        alt_idx = _find_any_col_index(header, ["Alt(m)", "Alt", "alt (m)", "altm"])
        fm_idx = _find_any_col_index(header, ["FM", "flightmode", "mode"])
        if gps_idx is None:
            meta["error"] = "No GPS column found."
            return [], stats, meta
        if sats_idx is None:
            meta["error"] = "No sats/Sats column found."
            return [], stats, meta
        if alt_idx is None:
            meta["error"] = "No Alt(m) column found."
            return [], stats, meta

        rows = list(reader)
        stats["rows"] = len(rows)

    input_type = getattr(takeoff_reference, "input_altitude_type", "csv_logic") if takeoff_reference else "csv_logic"
    used_takeoff = getattr(takeoff_reference, "override_takeoff_msl", None) if takeoff_reference else None
    csv_takeoff = getattr(takeoff_reference, "csv_takeoff_msl", None) if takeoff_reference else None

    # Pre-clean relative-altitude rows so MSL/elevation glitches are never written into the KMZ.
    cleaned_relative_by_row: Dict[int, float] = {}
    if input_type == "relative":
        raw_samples: List[Dict[str, Any]] = []
        for row_num, row in enumerate(rows):
            raw_alt = _parse_float(_clean_cell(row, alt_idx))
            if raw_alt is not None:
                raw_samples.append({"alt": float(raw_alt), "row": row_num})
        cleaned = _filter_relative_altitude_spikes(raw_samples)
        cleaned_relative_by_row = {int(s.get("row", -1)): float(s.get("alt", 0.0)) for s in cleaned}

    takeoff_msl: Optional[float] = used_takeoff
    if takeoff_msl is None and csv_takeoff is not None:
        takeoff_msl = float(csv_takeoff)
    relative_started = False

    def close_segment() -> None:
        nonlocal current_points, current_mode, last_key
        if current_points:
            segments.append(KMZSegment(mode=current_mode, points=current_points))
        current_points = []
        last_key = None

    for row_num, row in enumerate(rows):
        sv = _parse_float(_clean_cell(row, sats_idx))
        if sv is not None and int(round(float(sv))) == 4:
            stats["four_sat_rows_total"] += 1
        if sv is None or sv < float(min_sats):
            close_segment(); stats["low_sats"] += 1; continue
        if int(round(float(sv))) == 4:
            stats["four_sat_rows_kept"] += 1

        gps = _parse_gps_cell(_clean_cell(row, gps_idx))
        if gps is None:
            close_segment(); stats["missing_gps"] += 1; continue

        raw_alt = _parse_float(_clean_cell(row, alt_idx))
        if raw_alt is None:
            close_segment(); stats["missing_alt"] += 1; continue

        lat, lon = float(gps[0]), float(gps[1])

        if input_type == "asl":
            csv_ref = csv_takeoff
            if used_takeoff is not None and csv_ref is not None:
                alt_msl = float(used_takeoff) + (float(raw_alt) - float(csv_ref))
                meta["altitude_mode"] = "asl_csv_shifted_to_used_takeoff_alt"
            else:
                alt_msl = float(raw_alt)
                meta["altitude_mode"] = "csv_asl_absolute"
                meta.setdefault("used_takeoff_alt_m", f"{float(raw_alt):.2f}")

        elif input_type == "relative":
            if row_num not in cleaned_relative_by_row:
                close_segment()
                stats["missing_alt"] += 1
                continue
            if takeoff_msl is None:
                # Last-resort fallback: no takeoff reference somehow resolved.
                takeoff_msl = 0.0
                meta.setdefault("used_takeoff_alt_m", "0.00")
            rel_alt = cleaned_relative_by_row[row_num]
            alt_msl = float(takeoff_msl) + float(rel_alt)
            meta["altitude_mode"] = "relative_alt_plus_used_takeoff_alt"

        else:
            # CSV-only fallback: use the classic Betaflight MSL-then-relative behaviour,
            # but skip obvious MSL spikes when relative mode has started.
            if takeoff_msl is None:
                takeoff_msl = float(raw_alt)
                if abs(float(raw_alt)) >= KMZ_MSL_LOOKING_ALT_M:
                    meta.setdefault("used_takeoff_alt_m", f"{takeoff_msl:.2f}")
            if not relative_started and takeoff_msl is not None and abs(takeoff_msl) >= KMZ_MSL_LOOKING_ALT_M and abs(float(raw_alt)) <= KMZ_NEAR_ZERO_ALT_M:
                relative_started = True
            if relative_started and takeoff_msl is not None:
                # Avoid writing later MSL-looking spikes into CSV-only Betaflight output.
                if abs(float(raw_alt)) >= KMZ_MSL_LOOKING_ALT_M and abs(float(raw_alt) - float(takeoff_msl)) <= 100.0:
                    close_segment()
                    stats["missing_alt"] += 1
                    continue
                alt_msl = float(takeoff_msl) + float(raw_alt)
                meta["altitude_mode"] = "csv_betaflight_msl_then_relative"
            else:
                alt_msl = float(raw_alt)
                meta["altitude_mode"] = "csv_altitude_as_written"

        alt_msl = float(alt_msl) + float(visual_offset_m)

        mode = _strip_quotes(_clean_cell(row, fm_idx)) if fm_idx is not None else ""
        if current_points and mode != current_mode:
            close_segment()
        current_mode = mode

        key = (round(lat, KMZ_DEDUP_DECIMALS), round(lon, KMZ_DEDUP_DECIMALS))
        if last_key == key:
            stats["deduped"] += 1
            continue
        last_key = key

        current_points.append(KMZPoint3D(lat=lat, lon=lon, alt_msl=float(alt_msl)))
        stats["kept"] += 1

    close_segment()
    stats["segments"] = len(segments)
    meta["visual_vertical_offset_applied_m"] = f"{float(visual_offset_m):.2f}"
    meta["minimum_satellites_used"] = str(int(min_sats))
    return segments, stats, meta


def kmz_build_kml(segments: List[KMZSegment], meta: Dict[str, str]) -> str:
    first_pt: Optional[KMZPoint3D] = None
    for seg in segments:
        if seg.points:
            first_pt = seg.points[0]
            break
    placemarks: List[str] = []
    if first_pt is not None:
        coords0 = f"{first_pt.lon:.7f},{first_pt.lat:.7f},{first_pt.alt_msl:.2f}"
        placemarks.append(f"""      <Placemark>
        <name>0 Flight Path </name>
        <styleUrl>#yellowLineGreenPoly</styleUrl>
        <Style><LineStyle><color>{KMZ_ROYGBIV_KML[0]}</color><colorMode>normal</colorMode><width>4</width></LineStyle></Style>
        <LineString><extrude>1</extrude><altitudeMode>absolute</altitudeMode><coordinates>{coords0}</coordinates></LineString>
      </Placemark>""")
    seg_number = 1
    color_idx = 1
    for seg in segments:
        if len(seg.points) < 2:
            continue
        color = KMZ_ROYGBIV_KML[color_idx % len(KMZ_ROYGBIV_KML)]
        name = f"{seg_number} Flight Path  {seg.mode}".rstrip()
        coords = " ".join(f"{p.lon:.7f},{p.lat:.7f},{p.alt_msl:.2f}" for p in seg.points)
        placemarks.append(f"""      <Placemark>
        <name>{kmz_escape(name)}</name>
        <styleUrl>#yellowLineGreenPoly</styleUrl>
        <Style><LineStyle><color>{color}</color><colorMode>normal</colorMode><width>4</width></LineStyle></Style>
        <LineString><extrude>1</extrude><altitudeMode>absolute</altitudeMode><coordinates>{coords}</coordinates></LineString>
      </Placemark>""")
        seg_number += 1
        color_idx += 1
    meta_lines = "<br/>".join(f"{kmz_escape(k)}: {kmz_escape(v)}" for k, v in meta.items() if str(v).strip())
    meta_desc = f"<![CDATA[{meta_lines}]]>" if meta_lines else ""
    return f"""<?xml version="1.0"?>
<kml xmlns:xsd="http://www.w3.org/2001/XMLSchema" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <Document>
    <Style id="yellowLineGreenPoly"><LineStyle><color>7F00FFFF</color><colorMode>normal</colorMode><width>4</width></LineStyle><PolyStyle><color>7F00FF00</color><colorMode>normal</colorMode></PolyStyle></Style>
    <Folder>
      <name>Log</name>
      <description>{meta_desc}</description>
{os.linesep.join(placemarks)}
    </Folder>
  </Document>
</kml>
"""


def kmz_write_kmz(csv_path: str, altitude_options: KMZAltitudeOptions, output_suffix: str = "", manual_fallback_callback: Optional[Any] = None, confirm_callback: Optional[Any] = None, status_callback: Optional[Any] = None) -> Optional[str]:
    csv_path = os.path.abspath(csv_path)
    takeoff_reference = kmz_resolve_takeoff_reference(csv_path, altitude_options, manual_fallback_callback=manual_fallback_callback, confirm_callback=confirm_callback, status_callback=status_callback)
    min_sats = int(getattr(altitude_options, "min_sats", MIN_SATS) or MIN_SATS)
    segments, stats, meta = kmz_read_segments_from_csv(csv_path, takeoff_reference=takeoff_reference, visual_offset_m=altitude_options.visual_offset_m, min_sats=min_sats)
    total_points = sum(len(s.points) for s in segments)
    if total_points < 2:
        if status_callback:
            status_callback(f"No usable 3D flight path points found in {csv_path}. {meta.get('error', '')}")
        return None
    base = os.path.splitext(os.path.basename(csv_path))[0]
    out_path = os.path.join(os.path.dirname(csv_path), base + output_suffix + ".kmz")
    kml_name = base + output_suffix + ".log.kml"
    kml_text = kmz_build_kml(segments, meta)
    with zipfile.ZipFile(out_path, "w", compression=zipfile.ZIP_DEFLATED) as z:
        z.writestr(kml_name, kml_text)
    if status_callback:
        status_callback(f"Created KMZ: {out_path}")
        status_callback(f"  Segments: {len(segments)} | Points kept: {total_points} | Deduped: {stats['deduped']}")
        status_callback(f"  Breaks -> missing GPS: {stats['missing_gps']}, low sats(<{min_sats}): {stats['low_sats']}, missing alt: {stats['missing_alt']}")
        if min_sats <= RELAXED_MIN_SATS and int(stats.get("four_sat_rows_kept", 0) or 0) > 0:
            status_callback(f"  WARNING: Included {int(stats.get('four_sat_rows_kept', 0) or 0)} four-satellite row(s); position and GPS altitude may be less reliable.")
        elif min_sats > RELAXED_MIN_SATS and int(stats.get("four_sat_rows_total", 0) or 0) > 0:
            status_callback(f"  WARNING: Excluded {int(stats.get('four_sat_rows_total', 0) or 0)} four-satellite row(s); the 3D route may contain gaps.")
        status_callback(f"  Alt mode: {meta.get('altitude_mode')} | Takeoff source: {meta.get('takeoff_reference_source', 'n/a')} | Used takeoff alt: {meta.get('used_takeoff_alt_m', 'n/a')} m | Compensation: {meta.get('visual_vertical_offset_applied_m')} m")
    return out_path

# All flights summary + GUI additions
# ============================================================

def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return default
        return float(value)
    except Exception:
        return default


def _first_nonempty(values: List[str], default: str = "") -> str:
    for value in values:
        s = str(value or "").strip()
        if s:
            return s
    return default


def _parse_summary_paths_interactive() -> List[str]:
    """Prompt for one or more file/folder paths, one per line."""
    print("\nEnter CSV file or folder path(s).")
    print("Enter one path per line. Press Enter on a blank line when done.")
    paths: List[str] = []
    while True:
        raw = input("Path: ").strip()
        if not raw:
            break
        p = normalize_path(raw)
        if os.path.isfile(p) or os.path.isdir(p):
            paths.append(p)
        else:
            print("❌ Path not found. Try again.")
    return paths


def csv_files_from_paths(paths: List[str]) -> List[str]:
    """Expand a mixture of CSV files and folders into a sorted unique CSV file list."""
    found: List[str] = []
    seen = set()
    for p in paths:
        p = normalize_path(str(p))
        candidates: List[str] = []
        if os.path.isfile(p) and p.lower().endswith(".csv"):
            candidates = [p]
        elif os.path.isdir(p):
            candidates = find_csv_files(p)

        for csv_path in candidates:
            ap = os.path.abspath(csv_path)
            if ap not in seen:
                seen.add(ap)
                found.append(ap)

    found.sort()
    return found



def is_probable_flight_log_csv(csv_path: str) -> bool:
    """
    Return True when a CSV looks like an EdgeTX/Betaflight or ArduPilot-style flight log.

    This prevents unrelated CSV files in a folder, such as sports/statistics datasets,
    from being counted as zero-length aircraft in the all-flights summary.
    """
    try:
        with open(csv_path, "r", encoding="utf-8-sig", errors="replace", newline="") as f:
            dialect = sniff_dialect(f)
            reader = csv.reader(f, dialect)
            header = next(reader, None)
    except Exception:
        return False

    if not header:
        return False

    has_time = _find_col_index(header, "Time") is not None
    has_date = _find_col_index(header, "Date") is not None
    has_gps = _find_col_index(header, "GPS") is not None
    telemetry_targets = [
        "sats", "GSpd", "Alt", "Alt(m)", "alt (m)", "Capa", "Capa(mAh)",
        "Curr", "Curr(A)", "RxBt", "RxBt(V)", "RQly", "RSNR", "1RSS", "2RSS", "TPWR", "CH1", "CH1(us)"
    ]
    has_telemetry = _find_any_col_index(header, telemetry_targets) is not None

    return bool(has_time and (has_gps or has_telemetry or has_date and has_telemetry))


def _clean_aircraft_name_from_filename(csv_path: str) -> str:
    """Infer an aircraft/model name from a CSV filename when no model column exists."""
    base = os.path.splitext(os.path.basename(csv_path))[0]

    # Remove common date/time suffixes: -2025-05-22-121750, _2025-05-22_121750, etc.
    base = re.sub(r"[-_ ]?20\d{2}[-_ ]?\d{2}[-_ ]?\d{2}([-_ ]?\d{2,6}(\.\d+)?)?.*$", "", base)
    base = re.sub(r"[-_ ]?\d{8}[-_ ]?\d{4,6}.*$", "", base)

    # Remove map-output suffixes if a user accidentally points to renamed exports.
    base = re.sub(r"\s*\((default|detailed|osm|topo|contours|satellite|natgeo|layers)\)\s*$", "", base, flags=re.I)

    # Clean separators, but keep useful model characters.
    base = base.replace("_", " ").replace("-", " ").strip()
    base = re.sub(r"\s+", " ", base)

    if not base:
        parent = os.path.basename(os.path.dirname(csv_path)).strip()
        return parent or "Unknown aircraft"
    return base


def detect_aircraft_name(csv_path: str) -> str:
    """
    Try to detect aircraft/model name from CSV columns first, then filename.
    EdgeTX logs often do not include model name, so filename inference is important.
    """
    try:
        with open(csv_path, "r", encoding="utf-8-sig", errors="replace", newline="") as f:
            dialect = sniff_dialect(f)
            reader = csv.reader(f, dialect)
            header = next(reader, None)
            if not header:
                return _clean_aircraft_name_from_filename(csv_path)

            possible = [
                "model", "modelname", "model name", "aircraft", "aircraftname",
                "aircraft name", "vehicle", "vehiclename", "vehicle name", "craft"
            ]
            idx = _find_any_col_index(header, possible)
            if idx is not None:
                for row in reader:
                    if idx < len(row):
                        name = str(row[idx]).strip()
                        if name:
                            return name
    except Exception:
        pass

    return _clean_aircraft_name_from_filename(csv_path)


def _summary_date_parts(flight_stats: Dict[str, Any], csv_path: str) -> Tuple[str, str, str]:
    """
    Return (date_display, year, year_month) for all-flights summaries.

    Preference order:
      1) Date column from the CSV log, when available/parseable.
      2) Date pattern in the filename, as a fallback.
      3) Unknown.

    This keeps renamed/copied files from overriding the actual flight date stored in the log.
    """
    date_display = str(flight_stats.get("time", {}).get("date", "n/a") or "n/a")
    year = "Unknown"
    year_month = "Unknown"

    # First try the CSV Date column, which read_flight_data has already formatted like APR. 19, 2026.
    parsed_csv_date: Optional[datetime] = None
    if date_display and date_display.lower() != "n/a":
        for fmt in ("%b. %d, %Y", "%b %d, %Y", "%Y-%m-%d", "%Y/%m/%d", "%m/%d/%Y", "%d/%m/%Y"):
            try:
                parsed_csv_date = datetime.strptime(date_display.title(), fmt)
                break
            except Exception:
                pass
        if parsed_csv_date is None:
            m = re.search(r"\b(20\d{2})\b", date_display)
            if m:
                year = m.group(1)
                month_lookup = {
                    "JAN": "01", "FEB": "02", "MAR": "03", "APR": "04", "MAY": "05", "JUN": "06",
                    "JUL": "07", "AUG": "08", "SEP": "09", "OCT": "10", "NOV": "11", "DEC": "12",
                }
                mm = None
                for key, value in month_lookup.items():
                    if date_display.upper().startswith(key):
                        mm = value
                        break
                year_month = f"{year}-{mm}" if mm else year

    if parsed_csv_date is not None:
        year = str(parsed_csv_date.year)
        year_month = f"{parsed_csv_date.year:04d}-{parsed_csv_date.month:02d}"
        return date_display, year, year_month

    # Filename fallback only if the CSV Date column was missing/unparseable.
    base = os.path.basename(csv_path)
    fm = re.search(r"(20\d{2})[-_ ]?(\d{2})[-_ ]?(\d{2})", base)
    if fm:
        year = fm.group(1)
        year_month = f"{fm.group(1)}-{fm.group(2)}"
        if not date_display or date_display.lower() == "n/a":
            date_display = f"{fm.group(1)}-{fm.group(2)}-{fm.group(3)}"

    return date_display, year, year_month


def _estimate_gps_good_bad_time(csv_path: str, min_sats: int = MIN_SATS) -> Tuple[Optional[float], Optional[float]]:
    """
    Estimate time spent with GPS track meeting the existing map-quality rule:
    valid GPS and sats at or above the selected minimum.

    Uses row-to-row time deltas from the same CSV timeline used for flight duration.
    Normal intervals are classified by the row's GPS quality. Large positive time gaps
    are counted as no/low-GPS/unlogged time rather than being silently ignored.
    If no usable time data exists, returns (None, None).
    """
    rows: List[Tuple[float, bool]] = []
    date_used = False

    try:
        with open(csv_path, "r", encoding="utf-8-sig", errors="replace", newline="") as f:
            dialect = sniff_dialect(f)
            reader = csv.reader(f, dialect)
            header = next(reader, None)
            if not header:
                return None, None

            data_rows = [list(row) for row in reader]
            datetime_idx = _select_datetime_columns(header, data_rows)
            gps_idx = _find_col_index(header, "GPS")
            sats_idx = _best_numeric_col_index(header, data_rows, ["sats"])
            time_idx = datetime_idx.get("time")
            date_idx = datetime_idx.get("date")
            utc_idx = datetime_idx.get("utc_datetime")

            if time_idx is None and utc_idx is None:
                return None, None

            for row in data_rows:
                date_text = _clean_cell(row, date_idx)
                time_text = _clean_cell(row, time_idx)
                t = _parse_datetime_value(date_text, time_text) if time_idx is not None else None
                if t is None and utc_idx is not None:
                    utc_dt = _parse_combined_datetime_text(_clean_cell(row, utc_idx))
                    if utc_dt is not None:
                        # This estimator needs elapsed differences only.  Convert the
                        # sensor UTC clock to the same timezone-neutral ordinal seconds
                        # convention used by _parse_datetime_value().
                        t = (float(utc_dt.toordinal() * 86400 + utc_dt.hour * 3600 + utc_dt.minute * 60 + utc_dt.second)
                             + utc_dt.microsecond / 1_000_000.0)
                        date_used = True
                if t is None:
                    continue
                if date_text:
                    date_used = True

                gps_ok = False
                if gps_idx is not None:
                    gps_ok = _parse_gps_cell(_clean_cell(row, gps_idx)) is not None

                sats_ok = True
                if sats_idx is not None:
                    sats_val = _parse_sats(_clean_cell(row, sats_idx))
                    sats_ok = sats_val is not None and sats_val >= float(min_sats)

                rows.append((t, bool(gps_ok and sats_ok)))

        if len(rows) < 2:
            return None, None

        times = _unwrap_time_values([t for t, _ in rows], date_used)
        good_s = 0.0
        bad_s = 0.0
        for i in range(len(times) - 1):
            dt = times[i + 1] - times[i]
            if dt < 0:
                continue

            # Long gaps are not trusted as GPS-good logged intervals, even if the
            # previous row was good. Count them as no/low/unlogged GPS time.
            if dt > 60:
                bad_s += dt
                continue

            if rows[i][1]:
                good_s += dt
            else:
                bad_s += dt

        return good_s, bad_s
    except Exception:
        return None, None


def summarize_one_flight(csv_path: str, skip_non_flight_logs: bool = True, min_sats: int = MIN_SATS) -> Optional[Dict[str, Any]]:
    """Create one normalized summary record for a CSV flight log."""
    try:
        if skip_non_flight_logs and not is_probable_flight_log_csv(csv_path):
            return None
        aircraft_raw = detect_aircraft_name(csv_path)
        flight_data = read_flight_data(csv_path, min_sats=min_sats)
        flight_stats = flight_data.get("flight_stats", {})
        parse_stats = flight_data.get("parse_stats", {})
        numeric = flight_stats.get("numeric", {})
        time_info = flight_stats.get("time", {})
        dist = flight_stats.get("distance", {})
        altitude = flight_stats.get("altitude", {})
        efficiency = flight_stats.get("efficiency", {})
        autonomy = flight_stats.get("autonomy", {})
        flight_stack = flight_stats.get("flight_stack", {})

        gps_good_s, gps_bad_s = _estimate_gps_good_bad_time(csv_path, min_sats=min_sats)
        date_display, year, year_month = _summary_date_parts(flight_stats, csv_path)

        duration_s = _safe_float(time_info.get("duration_s"), 0.0)
        if duration_s <= 0:
            gps_good_s = 0.0
            gps_bad_s = 0.0
        elif gps_good_s is None or gps_bad_s is None:
            gps_good_s = 0.0
            gps_bad_s = duration_s
        else:
            gps_good_s = max(0.0, float(gps_good_s))
            gps_bad_s = max(0.0, float(gps_bad_s))
            gps_total = gps_good_s + gps_bad_s

            # Safety only: the estimator should already use the same first-to-last timeline.
            # If a malformed log makes the components differ slightly from duration, keep the
            # good GPS time from actual good intervals and put the remainder into no/low GPS.
            if abs(gps_total - duration_s) > 1.0:
                gps_good_s = min(gps_good_s, duration_s)
                gps_bad_s = max(0.0, duration_s - gps_good_s)

        record = {
            "path": csv_path,
            "filename": os.path.basename(csv_path),
            "aircraft_raw": aircraft_raw,
            "aircraft": aircraft_raw,
            "date": date_display,
            "start_time": _format_summary_start_time(time_info.get("takeoff")),
            "year": year,
            "year_month": year_month,
            "duration_s": duration_s,
            "gps_good_s": gps_good_s,
            "gps_bad_s": gps_bad_s,
            "distance_m": _safe_float(dist.get("total_m"), 0.0),
            "max_home_m": _safe_float(dist.get("max_home_m"), 0.0),
            "capacity_used_mAh": _safe_float(efficiency.get("capacity_used_mAh"), 0.0),
            "max_gspd_kmh": _safe_float(numeric.get("gspd", {}).get("max"), 0.0),
            "avg_gspd_kmh": _safe_float(numeric.get("gspd", {}).get("avg"), 0.0),
            "max_alt_agl_m": _safe_float(altitude.get("alt"), 0.0),
            "takeoff_elevation_m": altitude.get("takeoff_elevation_m"),
            "avg_rsnr": numeric.get("rsnr", {}).get("avg"),
            "min_rqly": numeric.get("rqly", {}).get("min"),
            "min_rssi_dbm": numeric.get("rssi_best", {}).get("min"),
            "rows": int(parse_stats.get("rows", 0) or 0),
            "gps_points": int(parse_stats.get("kept", 0) or 0),
            "low_sats_rows": int(parse_stats.get("low_sats", 0) or 0),
            "gps_gap_rows": int(parse_stats.get("missing_gps", 0) or 0),
            "altitude_source": altitude.get("source", "unknown"),
            "gps_min_sats": int(min_sats),
            "four_sat_rows_total": int(parse_stats.get("four_sat_rows_total", 0) or 0),
            "four_sat_rows_kept": int(parse_stats.get("four_sat_rows_kept", 0) or 0),
            "flight_stack": str(flight_stack.get("stack", "unknown")),
            "flight_stack_confidence": str(flight_stack.get("confidence", "low")),
            "control_classification": str(autonomy.get("classification", "unknown")),
            "semi_autonomous_fraction": float(autonomy.get("autonomous_fraction", 0.0) or 0.0),
            "semi_autonomous_modes": list(autonomy.get("autonomous_modes", []) or []),
        }
        return record
    except Exception as exc:
        print(f"⚠️  Could not summarize {csv_path}: {exc}")
        return None


def scan_flights_for_summary(csv_files: List[str], skip_non_flight_logs: bool = True, min_sats: int = MIN_SATS) -> List[Dict[str, Any]]:
    """Summarize a list of CSV files, optionally skipping files whose headers do not look like flight logs."""
    records: List[Dict[str, Any]] = []
    skipped = 0
    for i, csv_path in enumerate(csv_files, start=1):
        print(f"Summarizing {i}/{len(csv_files)}: {os.path.basename(csv_path)}")
        rec = summarize_one_flight(csv_path, skip_non_flight_logs=skip_non_flight_logs, min_sats=min_sats)
        if rec is not None:
            records.append(rec)
        else:
            skipped += 1
    if skipped:
        if skip_non_flight_logs:
            print(f"Skipped {skipped} CSV file(s) that did not look like flight logs. Turn off the skip option to force-include them.")
        else:
            print(f"Skipped {skipped} CSV file(s) because they could not be read/summarized.")
    return records


def _suggest_aircraft_groups(raw_names: List[str]) -> Dict[str, str]:
    """Auto-suggest aircraft grouping based on similar names."""
    mapping: Dict[str, str] = {name: name for name in raw_names}
    names = list(raw_names)
    used = set()
    for name in names:
        if name in used:
            continue
        group = [name]
        for other in names:
            if other == name or other in used:
                continue
            ratio = difflib.SequenceMatcher(None, name.lower(), other.lower()).ratio()
            if ratio >= 0.78:
                group.append(other)
        if len(group) > 1:
            canonical = min(group, key=len)
            for item in group:
                mapping[item] = canonical
                used.add(item)
    return mapping


def prompt_aircraft_grouping(records: List[Dict[str, Any]]) -> Dict[str, str]:
    """Ask user to map raw detected names into aircraft groups."""
    raw_names = sorted({r["aircraft_raw"] for r in records})
    suggested = _suggest_aircraft_groups(raw_names)
    remembered = load_aircraft_group_mapping()

    print("\nDetected aircraft/model names:")
    for i, name in enumerate(raw_names, start=1):
        default_group = remembered.get(name, suggested.get(name, name))
        note = "" if default_group == name else f"  group: {default_group}"
        print(f"{i}) {name}{note}")

    print("\nGroup names for aircraft totals.")
    print("Press Enter to accept the shown/suggested group name, type a corrected aircraft name, or type skip to exclude that raw name.")
    mapping: Dict[str, str] = {}
    for name in raw_names:
        default = remembered.get(name, suggested.get(name, name))
        raw = input(f"Group for '{name}' [Enter = {default}]: ").strip()
        if raw.lower() in ("skip", "exclude", "ignore", "delete", "remove"):
            continue
        mapping[name] = raw if raw else default

    if mapping:
        save_aircraft_group_mapping(mapping, merge=True)
    return mapping


def apply_aircraft_mapping(records: List[Dict[str, Any]], mapping: Dict[str, str], only_mapped: bool = False) -> List[Dict[str, Any]]:
    """Apply raw aircraft name -> canonical group mapping."""
    updated: List[Dict[str, Any]] = []
    for rec in records:
        raw_name = str(rec.get("aircraft_raw", ""))
        if only_mapped and raw_name not in mapping:
            continue
        group_name = mapping.get(raw_name, raw_name or "Unknown aircraft")
        group_name = str(group_name).strip()
        if not group_name:
            continue
        new = dict(rec)
        new["aircraft"] = group_name
        updated.append(new)
    return updated


def _empty_bucket() -> Dict[str, Any]:
    return {
        "flights": 0,
        "duration_s": 0.0,
        "gps_good_s": 0.0,
        "gps_bad_s": 0.0,
        "distance_m": 0.0,
        "capacity_used_mAh": 0.0,
        "max_gspd_kmh": 0.0,
        "max_gspd_source": "",
        "max_alt_agl_m": 0.0,
        "max_alt_source": "",
        "max_home_m": 0.0,
        "max_home_source": "",
        "gps_points": 0,
        "low_sats_rows": 0,
        "gps_gap_rows": 0,
        "four_sat_rows_total": 0,
        "four_sat_rows_kept": 0,
        "gps_min_sats": MIN_SATS,
        "best_avg_rsnr": None,
        "lowest_rssi_dbm": None,
        "semi_autonomous_flights": 0,
        "controller_managed_time_s": 0.0,
    }


def _format_summary_start_time(time_text: Any) -> str:
    """Format a CSV start time like 09:15:44.190 as 9:15 am for report source labels."""
    raw = str(time_text or "").strip()
    if not raw or raw.lower() == "n/a":
        return ""
    for fmt in ("%H:%M:%S.%f", "%H:%M:%S", "%H:%M"):
        try:
            dt = datetime.strptime(raw, fmt)
            return dt.strftime("%I:%M %p").lstrip("0").lower()
        except Exception:
            pass
    m = re.search(r"(\d{1,2}):(\d{2})", raw)
    if m:
        try:
            h = int(m.group(1))
            minute = int(m.group(2))
            ampm = "am" if h < 12 else "pm"
            h12 = h % 12 or 12
            return f"{h12}:{minute:02d} {ampm}"
        except Exception:
            pass
    return ""


def _summary_source_label(rec: Dict[str, Any]) -> str:
    date = str(rec.get("date", "n/a") or "n/a")
    start_time = str(rec.get("start_time", "") or "")
    aircraft = str(rec.get("aircraft", rec.get("aircraft_raw", "Unknown")) or "Unknown")
    if start_time:
        return f"{date}, {start_time}, {aircraft}"
    return f"{date}, {aircraft}"


def _format_with_source(value_text: str, source: str) -> str:
    return f"{value_text} ({source})" if source else value_text


def _add_to_bucket(bucket: Dict[str, Any], rec: Dict[str, Any]) -> None:
    bucket["flights"] += 1
    bucket["duration_s"] += _safe_float(rec.get("duration_s"))
    if rec.get("gps_good_s") is not None:
        bucket["gps_good_s"] += _safe_float(rec.get("gps_good_s"))
    if rec.get("gps_bad_s") is not None:
        bucket["gps_bad_s"] += _safe_float(rec.get("gps_bad_s"))
    bucket["distance_m"] += _safe_float(rec.get("distance_m"))
    bucket["capacity_used_mAh"] += _safe_float(rec.get("capacity_used_mAh"))
    if str(rec.get("control_classification", "")) == "semi_autonomous":
        bucket["semi_autonomous_flights"] += 1
        bucket["controller_managed_time_s"] += _safe_float(rec.get("duration_s")) * max(0.0, min(1.0, _safe_float(rec.get("semi_autonomous_fraction"))))

    gspd = _safe_float(rec.get("max_gspd_kmh"))
    if bucket["flights"] == 1 or gspd > bucket["max_gspd_kmh"]:
        bucket["max_gspd_kmh"] = gspd
        bucket["max_gspd_source"] = _summary_source_label(rec)

    alt = _safe_float(rec.get("max_alt_agl_m"))
    if bucket["flights"] == 1 or alt > bucket["max_alt_agl_m"]:
        bucket["max_alt_agl_m"] = alt
        bucket["max_alt_source"] = _summary_source_label(rec)

    home = _safe_float(rec.get("max_home_m"))
    if bucket["flights"] == 1 or home > bucket["max_home_m"]:
        bucket["max_home_m"] = home
        bucket["max_home_source"] = _summary_source_label(rec)

    bucket["gps_points"] += int(rec.get("gps_points", 0) or 0)
    bucket["low_sats_rows"] += int(rec.get("low_sats_rows", 0) or 0)
    bucket["gps_gap_rows"] += int(rec.get("gps_gap_rows", 0) or 0)
    bucket["four_sat_rows_total"] += int(rec.get("four_sat_rows_total", 0) or 0)
    bucket["four_sat_rows_kept"] += int(rec.get("four_sat_rows_kept", 0) or 0)
    bucket["gps_min_sats"] = min(int(bucket.get("gps_min_sats", MIN_SATS) or MIN_SATS), int(rec.get("gps_min_sats", MIN_SATS) or MIN_SATS))

    avg_rsnr = rec.get("avg_rsnr")
    if isinstance(avg_rsnr, (int, float)):
        if bucket["best_avg_rsnr"] is None or avg_rsnr > bucket["best_avg_rsnr"]:
            bucket["best_avg_rsnr"] = float(avg_rsnr)

    rssi = rec.get("min_rssi_dbm")
    if isinstance(rssi, (int, float)):
        if bucket["lowest_rssi_dbm"] is None or rssi < bucket["lowest_rssi_dbm"]:
            bucket["lowest_rssi_dbm"] = float(rssi)


def aggregate_records(records: List[Dict[str, Any]], key: str) -> Dict[str, Dict[str, Any]]:
    buckets: Dict[str, Dict[str, Any]] = {}
    for rec in records:
        group = str(rec.get(key, "Unknown") or "Unknown")
        if group not in buckets:
            buckets[group] = _empty_bucket()
        _add_to_bucket(buckets[group], rec)
    return dict(sorted(buckets.items(), key=lambda item: item[0]))


def _format_summary_bucket(name: str, bucket: Dict[str, Any], indent: str = "") -> List[str]:
    max_speed_text = _format_with_source(f"{bucket['max_gspd_kmh']:.1f} km/h", bucket.get("max_gspd_source", ""))
    max_alt_text = _format_with_source(f"{bucket['max_alt_agl_m']:.1f} m", bucket.get("max_alt_source", ""))
    max_home_text = _format_with_source(_format_distance(bucket["max_home_m"]), bucket.get("max_home_source", ""))
    lines = [
        f"{indent}{name}",
        f"{indent}  Flights: {bucket['flights']}",
        f"{indent}  Total flight time: {_format_duration(bucket['duration_s'])}",
        f"{indent}  GPS-good logged time: {_format_duration(bucket['gps_good_s'])}",
        f"{indent}  No/low-GPS time: {_format_duration(bucket['gps_bad_s'])}",
        f"{indent}  GPS-good threshold: {int(bucket.get('gps_min_sats', MIN_SATS))}+ satellites",
        f"{indent}  Total distance: {bucket['distance_m'] / 1000.0:.3f} km",
        f"{indent}  Total mAh used: {bucket['capacity_used_mAh']:.0f} mAh",
        f"{indent}  Semi-autonomous/controller-managed flights: {int(bucket.get('semi_autonomous_flights', 0))}",
        f"{indent}  Controller-managed logged time: {_format_duration(bucket.get('controller_managed_time_s', 0.0))}",
        f"{indent}  Max ground speed: {max_speed_text}",
        f"{indent}  Max relative altitude: {max_alt_text}",
        f"{indent}  Max distance from home: {max_home_text}",
    ]
    return lines


def build_all_flights_summary_report(records: List[Dict[str, Any]], title: str = "All Flights Summary", month_filter: str = "") -> str:
    """Build a readable text report for all summarized flight records."""
    filtered = list(records)
    month_filter = (month_filter or "").strip()
    if month_filter:
        filtered = [r for r in filtered if str(r.get("year_month", "")).startswith(month_filter) or str(r.get("year", "")) == month_filter]

    overall = _empty_bucket()
    for rec in filtered:
        _add_to_bucket(overall, rec)

    lines: List[str] = []
    lines.append(title)
    lines.append("=" * len(title))
    if month_filter:
        lines.append(f"Filter: {month_filter}")
    lines.append(f"Flights processed: {overall['flights']}")
    lines.append(f"Total flight time: {_format_duration(overall['duration_s'])}")
    lines.append(f"GPS-good logged time: {_format_duration(overall['gps_good_s'])}")
    lines.append(f"No/low-GPS time: {_format_duration(overall['gps_bad_s'])}")
    lines.append(f"GPS-good threshold: {int(overall.get('gps_min_sats', MIN_SATS))}+ satellites")
    if int(overall.get("four_sat_rows_total", 0) or 0) > 0:
        if int(overall.get("gps_min_sats", MIN_SATS) or MIN_SATS) <= RELAXED_MIN_SATS:
            lines.append(f"GPS warning: {int(overall.get('four_sat_rows_kept', 0) or 0)} four-satellite row(s) were included; position and GPS altitude may be less reliable.")
        else:
            lines.append(f"GPS warning: {int(overall.get('four_sat_rows_total', 0) or 0)} four-satellite row(s) were excluded; GPS distance/altitude coverage may contain gaps.")
    lines.append(f"Total distance: {overall['distance_m'] / 1000.0:.3f} km")
    lines.append(f"Total mAh used: {overall['capacity_used_mAh']:.0f} mAh")
    lines.append(f"Semi-autonomous/controller-managed flights: {int(overall.get('semi_autonomous_flights', 0))}")
    lines.append(f"Controller-managed logged time: {_format_duration(overall.get('controller_managed_time_s', 0.0))}")
    lines.append(f"Max ground speed: {_format_with_source(f'{overall['max_gspd_kmh']:.1f} km/h', overall.get('max_gspd_source', ''))}")
    lines.append(f"Max relative altitude: {_format_with_source(f'{overall['max_alt_agl_m']:.1f} m', overall.get('max_alt_source', ''))}")
    lines.append(f"Max distance from home: {_format_with_source(_format_distance(overall['max_home_m']), overall.get('max_home_source', ''))}")
    lines.append("")

    per_aircraft = aggregate_records(filtered, "aircraft")
    lines.append("Per-aircraft totals:")
    if not per_aircraft:
        lines.append("  No flights found.")
    for name, bucket in per_aircraft.items():
        lines.extend(_format_summary_bucket(f"[{name}]", bucket, indent="  "))
        lines.append("")

    per_year = aggregate_records(filtered, "year")
    lines.append("Per-year breakdown:")
    if not per_year:
        lines.append("  No flights found.")
    for name, bucket in per_year.items():
        lines.extend(_format_summary_bucket(f"[{name}]", bucket, indent="  "))
        lines.append("")

    per_month = aggregate_records(filtered, "year_month")
    lines.append("Per-month breakdown:")
    if not per_month:
        lines.append("  No flights found.")
    for name, bucket in per_month.items():
        lines.extend(_format_summary_bucket(f"[{name}]", bucket, indent="  "))
        lines.append("")

    lines.append("Individual flights:")
    for rec in sorted(filtered, key=lambda r: (str(r.get("date", "")), str(r.get("filename", "")))):
        control_note = ""
        if str(rec.get("control_classification", "")) == "semi_autonomous":
            modes = ",".join(str(x) for x in rec.get("semi_autonomous_modes", []) if str(x)) or "controller-managed"
            control_note = f" | semi-autonomous {float(rec.get('semi_autonomous_fraction', 0.0) or 0.0) * 100.0:.1f}% ({modes})"
        lines.append(
            f"  {rec.get('date', 'n/a')} {('(' + str(rec.get('start_time')) + ')') if rec.get('start_time') else ''} | {rec.get('aircraft', 'Unknown')} | {rec.get('filename')} | "
            f"time {_format_duration(rec.get('duration_s'))} | dist {rec.get('distance_m', 0.0)/1000.0:.3f} km | "
            f"mAh {rec.get('capacity_used_mAh', 0.0):.0f} | max speed {rec.get('max_gspd_kmh', 0.0):.1f} km/h | "
            f"max alt {rec.get('max_alt_agl_m', 0.0):.1f} m{control_note}"
        )

    return "\n".join(lines).rstrip() + "\n"


def save_summary_report_interactive(report: str, default_folder: Optional[str] = None) -> None:
    """Ask whether to save a summary report as a .txt file."""
    if not _ask_yes_no("Save this summary as a text file?", default=True):
        return
    default_folder = default_folder or os.getcwd()
    default_path = os.path.join(default_folder, "all_flights_summary.txt")
    raw = input(f"Output text file path (Enter = {default_path}): ").strip()
    out_path = normalize_path(raw) if raw else default_path
    try:
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(report)
        print(f"✅ Summary saved: {out_path}")
    except Exception as exc:
        print(f"❌ Could not save summary: {exc}")


def process_all_flights_summary_cli() -> None:
    """Console workflow for all-flights summary."""
    paths = _parse_summary_paths_interactive()
    if not paths:
        print("No paths entered.")
        return

    csv_files = csv_files_from_paths(paths)
    if not csv_files:
        print("⚠️  No CSV files found in the provided path(s).")
        return

    print(f"\nFound {len(csv_files)} CSV file(s).")
    skip_non_flight_logs = _ask_yes_no("Skip CSV files that do not look like flight logs? Recommended when scanning broad folders.", default=True)
    include_four_sat = _ask_yes_no("Include GPS rows with exactly 4 satellites? Five satellites remains recommended.", default=False)
    summary_min_sats = RELAXED_MIN_SATS if include_four_sat else MIN_SATS
    records = scan_flights_for_summary(csv_files, skip_non_flight_logs=skip_non_flight_logs, min_sats=summary_min_sats)
    if not records:
        print("⚠️  No usable flight records could be summarized.")
        return

    mapping = prompt_aircraft_grouping(records)
    records = apply_aircraft_mapping(records, mapping)

    print("\nOptional date filter:")
    print("Press Enter for all flights, type a year like 2025, or type a month like 2025-07.")
    month_filter = input("Filter: ").strip()

    report = build_all_flights_summary_report(records, month_filter=month_filter)
    print("\n" + report)
    save_summary_report_interactive(report, default_folder=os.path.dirname(csv_files[0]))


def _gui_try_enable_dpi_awareness() -> None:
    """Make Tk sharper on Windows high-DPI displays when possible."""
    if sys.platform.startswith("win"):
        try:
            import ctypes
            try:
                ctypes.windll.shcore.SetProcessDpiAwareness(1)
            except Exception:
                ctypes.windll.user32.SetProcessDPIAware()
        except Exception:
            pass


class _TextRedirector:
    """Redirect print output to a Tk text widget."""
    def __init__(self, widget: Any):
        self.widget = widget

    def write(self, text_value: str) -> None:
        if not text_value:
            return
        try:
            self.widget.insert("end", text_value)
            self.widget.see("end")
            self.widget.update_idletasks()
        except Exception:
            pass

    def flush(self) -> None:
        pass



def move_paths_to_recycle_bin(paths: List[str], status_callback: Optional[Any] = None) -> Tuple[int, int]:
    """
    Move the exact supplied files to the Windows Recycle Bin.

    Returns (moved_count, skipped_count). It never scans folders and never deletes
    anything except the paths provided by the current app session.
    """
    moved = 0
    skipped = 0

    def status(msg: str) -> None:
        if status_callback:
            try:
                status_callback(msg)
            except Exception:
                pass

    existing: List[str] = []
    seen = set()
    for p in paths:
        ap = os.path.abspath(str(p))
        if ap in seen:
            continue
        seen.add(ap)
        if os.path.isfile(ap):
            existing.append(ap)
        else:
            skipped += 1
            status(f"Skipped missing KMZ: {ap}")

    if not existing:
        return 0, skipped

    if sys.platform.startswith("win"):
        try:
            import ctypes
            from ctypes import wintypes

            class SHFILEOPSTRUCTW(ctypes.Structure):
                _fields_ = [
                    ("hwnd", wintypes.HWND),
                    ("wFunc", wintypes.UINT),
                    ("pFrom", wintypes.LPCWSTR),
                    ("pTo", wintypes.LPCWSTR),
                    ("fFlags", wintypes.USHORT),
                    ("fAnyOperationsAborted", wintypes.BOOL),
                    ("hNameMappings", wintypes.LPVOID),
                    ("lpszProgressTitle", wintypes.LPCWSTR),
                ]

            FO_DELETE = 0x0003
            FOF_ALLOWUNDO = 0x0040
            FOF_NOCONFIRMATION = 0x0010
            FOF_SILENT = 0x0004
            FOF_NOERRORUI = 0x0400

            # SHFileOperation expects a double-null-terminated list.
            from_list = "\0".join(existing) + "\0\0"
            op = SHFILEOPSTRUCTW()
            op.wFunc = FO_DELETE
            op.pFrom = from_list
            op.fFlags = FOF_ALLOWUNDO | FOF_NOCONFIRMATION | FOF_SILENT | FOF_NOERRORUI
            result = ctypes.windll.shell32.SHFileOperationW(ctypes.byref(op))
            if result == 0:
                moved = len(existing)
                for p in existing:
                    status(f"Moved to Recycle Bin: {p}")
                return moved, skipped
            status(f"Recycle Bin move failed with Windows code {result}; no fallback delete was attempted.")
            return 0, skipped + len(existing)
        except Exception as exc:
            status(f"Recycle Bin move failed: {exc}; no fallback delete was attempted.")
            return 0, skipped + len(existing)

    # Non-Windows fallback: only use send2trash if installed. Otherwise skip safely.
    try:
        from send2trash import send2trash  # type: ignore
        for p in existing:
            send2trash(p)
            moved += 1
            status(f"Moved to trash: {p}")
    except Exception as exc:
        status(f"Trash move unavailable on this system: {exc}; no fallback delete was attempted.")
        skipped += len(existing) - moved

    return moved, skipped

def launch_gui() -> int:
    """
    Full Tkinter GUI wrapper for the map-generation, data-analysis, and all-flights summary tools.

    The GUI exposes the single CSV, recursive CSV, data-analysis, all-flights summary,
    preset, basemap, stats, privacy, and stats-line removal workflows without requiring
    the console for normal use.
    """
    _gui_try_enable_dpi_awareness()

    try:
        import tkinter as tk
        from tkinter import filedialog, messagebox, simpledialog, ttk
    except Exception as exc:
        print(f"⚠️  GUI could not start because Tkinter is unavailable: {exc}")
        return main_console()

    try:
        from tkinterdnd2 import TkinterDnD, DND_FILES  # type: ignore
        root = TkinterDnD.Tk()
        dnd_available = True
    except Exception:
        root = tk.Tk()
        DND_FILES = None
        dnd_available = False

    APP_VERSION = APP_VERSION_NUMBER
    APP_BG = "#f2f2f2"
    PANEL_BG = "#f2f2f2"
    WHITE = "#ffffff"

    root.title(f"Flight Map Tools {APP_VERSION}")
    root.geometry("1260x900")
    root.minsize(980, 720)
    root.configure(bg=APP_BG)

    try:
        root._fpv_maple_icon = tk.PhotoImage(data=MAPLE_LEAF_ICON_PNG_BASE64, format="png")
        root.iconphoto(True, root._fpv_maple_icon)
    except Exception:
        pass


    try:
        root.option_add("*Font", "{Segoe UI} 10")
        root.option_add("*Background", APP_BG)
        root.option_add("*Entry.Background", WHITE)
        root.option_add("*Text.Background", WHITE)
    except Exception:
        pass

    style = ttk.Style(root)
    for theme_name in ("vista", "xpnative", "default", "clam"):
        try:
            if theme_name in style.theme_names():
                style.theme_use(theme_name)
                break
        except Exception:
            pass
    try:
        style.configure("TFrame", background=APP_BG)
        style.configure("TLabelframe", background=APP_BG)
        style.configure("TLabelframe.Label", background=APP_BG)
        style.configure("TLabel", background=APP_BG)
        style.configure("TCheckbutton", background=APP_BG)
        style.configure("TRadiobutton", background=APP_BG)
        style.configure("TEntry", fieldbackground=WHITE, background=WHITE)
        style.configure("TCombobox", fieldbackground=WHITE, background=WHITE)
        style.map("TCombobox", fieldbackground=[("readonly", WHITE), ("!disabled", WHITE)], background=[("readonly", WHITE), ("!disabled", WHITE)])
        style.configure("Vertical.TScrollbar", width=18, arrowsize=18)
    except Exception:
        pass

    # ------------------------------------------------------------------
    # GUI helpers
    # ------------------------------------------------------------------
    def log(msg: str) -> None:
        log_text.insert("end", msg + "\n")
        log_text.see("end")
        root.update_idletasks()

    def _parse_dropped_paths(data: str) -> List[str]:
        data = str(data or "").strip()
        parts = re.findall(r"\{([^}]+)\}|([^\s]+)", data)
        parsed = [a or b for a, b in parts]
        return parsed or ([data] if data else [])

    def add_dnd_to_entry(widget: Any, var: Any) -> None:
        if not dnd_available:
            return
        try:
            widget.drop_target_register(DND_FILES)
            def on_drop(event: Any) -> None:
                paths = _parse_dropped_paths(str(event.data))
                if paths:
                    var.set(paths[0])
            widget.dnd_bind("<<Drop>>", on_drop)
        except Exception:
            pass

    def add_dnd_to_text(widget: Any) -> None:
        if not dnd_available:
            return
        try:
            widget.drop_target_register(DND_FILES)
            def on_drop(event: Any) -> None:
                for p in _parse_dropped_paths(str(event.data)):
                    widget.insert("end", p + "\n")
            widget.dnd_bind("<<Drop>>", on_drop)
        except Exception:
            pass

    def _mark_mousewheel(widget: Any, target: Any) -> None:
        try:
            setattr(widget, "_fpv_mousewheel_target", target)
        except Exception:
            pass

    def _mousewheel_units(event: Any) -> int:
        if hasattr(event, "delta") and event.delta:
            return -1 * int(event.delta / 120) if abs(event.delta) >= 120 else (-1 if event.delta > 0 else 1)
        if getattr(event, "num", None) == 4:
            return -1
        if getattr(event, "num", None) == 5:
            return 1
        return 0

    def _scroll_target_with_chain(widget: Any, target: Any, units: int) -> str:
        try:
            first,last=target.yview()
            at_edge=(units<0 and first<=0.000001) or (units>0 and last>=0.999999)
            if not at_edge:
                target.yview_scroll(units,"units")
                return "break"
            scan=getattr(widget,"master",None)
            while scan is not None:
                outer=getattr(scan,"_fpv_mousewheel_target",None)
                if outer is not None and outer is not target:
                    outer.yview_scroll(units,"units")
                    return "break"
                scan=getattr(scan,"master",None)
        except Exception:
            pass
        return "break"

    def _global_mousewheel(event: Any) -> str:
        try:
            widget = root.winfo_containing(event.x_root, event.y_root)
            scan = widget
            # Normal widgets/text areas scroll themselves or their marked page target.
            # Comboboxes get their own widget-level binding below so the combo value
            # does not change from the mouse wheel; the surrounding page scrolls instead.
            while widget is not None:
                target = getattr(widget, "_fpv_mousewheel_target", None)
                if target is not None:
                    units = _mousewheel_units(event)
                    if units:
                        return _scroll_target_with_chain(scan, target, units)
                widget = getattr(widget, "master", None)
        except Exception:
            pass
        return ""

    root.bind_all("<MouseWheel>", _global_mousewheel)
    root.bind_all("<Button-4>", _global_mousewheel)
    root.bind_all("<Button-5>", _global_mousewheel)

    def _scroll_parent_from_widget(widget: Any, event: Any) -> str:
        """Scroll the nearest marked parent instead of changing a dropdown value."""
        units = _mousewheel_units(event)
        if not units:
            return "break"
        scan = getattr(widget, "master", None)
        while scan is not None:
            target = getattr(scan, "_fpv_mousewheel_target", None)
            if target is not None:
                return _scroll_target_with_chain(widget, target, units)
            scan = getattr(scan, "master", None)
        return "break"

    def make_combobox(parent: Any, *args: Any, **kwargs: Any) -> Any:
        """Create a readonly combobox whose mouse wheel scrolls the page, not the combo selection."""
        kwargs.setdefault("state", "readonly")
        combo = ttk.Combobox(parent, *args, **kwargs)
        try:
            combo.configure(cursor="arrow")
        except Exception:
            pass
        def _open_combo(event: Any, w: Any = combo) -> str:
            try:
                w.focus_set()
                w.event_generate("<Down>")
            except Exception:
                pass
            return "break"
        combo.bind("<Button-1>", _open_combo, add=False)
        combo.bind("<MouseWheel>", lambda event, w=combo: _scroll_parent_from_widget(w, event), add=False)
        combo.bind("<Button-4>", lambda event, w=combo: _scroll_parent_from_widget(w, event), add=False)
        combo.bind("<Button-5>", lambda event, w=combo: _scroll_parent_from_widget(w, event), add=False)
        return combo

    try:
        root.bind_class("TCombobox", "<MouseWheel>", lambda event: _scroll_parent_from_widget(event.widget, event), add=False)
        root.bind_class("TCombobox", "<Button-4>", lambda event: _scroll_parent_from_widget(event.widget, event), add=False)
        root.bind_class("TCombobox", "<Button-5>", lambda event: _scroll_parent_from_widget(event.widget, event), add=False)
    except Exception:
        pass

    def make_wrapped_label(parent: Any, text: str, **kwargs: Any) -> Any:
        """Create a label that wraps instead of running off the window edge."""
        kwargs.setdefault("justify", "left")
        kwargs.setdefault("wraplength", 900)
        lbl = ttk.Label(parent, text=text, **kwargs)
        def _update_wrap(event: Any = None, w: Any = lbl, p: Any = parent) -> None:
            try:
                width = max(220, int(p.winfo_width()) - 40)
                w.configure(wraplength=width)
            except Exception:
                pass
        try:
            parent.bind("<Configure>", _update_wrap, add="+")
            root.after_idle(_update_wrap)
        except Exception:
            pass
        return lbl

    def make_vscroll(parent: Any, command: Any) -> Any:
        try:
            return tk.Scrollbar(parent, orient="vertical", command=command, width=18)
        except Exception:
            return ttk.Scrollbar(parent, orient="vertical", command=command, style="Vertical.TScrollbar")

    def make_scrolled_frame(parent: Any, height: Optional[int] = None) -> Tuple[Any, Any, Any]:
        outer = ttk.Frame(parent)
        canvas = tk.Canvas(outer, highlightthickness=0, background=APP_BG, borderwidth=0, height=height or 0)
        scroll = make_vscroll(outer, canvas.yview)
        inner = ttk.Frame(canvas)
        window_id = canvas.create_window((0, 0), window=inner, anchor="nw")

        def _update_scrollregion(_event: Any = None) -> None:
            bbox = canvas.bbox("all") or (0, 0, 0, 0)
            content_h = max(0, int(bbox[3] - bbox[1]))
            view_h = max(1, int(canvas.winfo_height()))
            view_w = max(1, int(canvas.winfo_width()))
            # If content is shorter than the viewport, make the scrollregion equal
            # to the viewport so the page cannot scroll into empty blank space.
            if content_h <= view_h:
                canvas.configure(scrollregion=(0, 0, view_w, view_h))
                canvas.yview_moveto(0)
            else:
                canvas.configure(scrollregion=bbox)

        inner.bind("<Configure>", _update_scrollregion)
        try:
            setattr(inner, "_fpv_update_scrollregion", _update_scrollregion)
            setattr(outer, "_fpv_update_scrollregion", _update_scrollregion)
        except Exception:
            pass
        canvas.configure(yscrollcommand=scroll.set)
        def _on_canvas_configure(e: Any) -> None:
            canvas.itemconfigure(window_id, width=e.width)
            _update_scrollregion()
        canvas.bind("<Configure>", _on_canvas_configure)
        canvas.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        _mark_mousewheel(outer, canvas)
        _mark_mousewheel(canvas, canvas)
        _mark_mousewheel(inner, canvas)
        return outer, inner, canvas

    def make_scrolled_text(parent: Any, height: int = 8, wrap: str = "word") -> Tuple[Any, Any]:
        frame = ttk.Frame(parent)
        text_widget = tk.Text(frame, height=height, wrap=wrap, background=WHITE, relief="sunken", borderwidth=1)
        scroll = make_vscroll(frame, text_widget.yview)
        text_widget.configure(yscrollcommand=scroll.set)
        text_widget.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        _mark_mousewheel(frame, text_widget)
        _mark_mousewheel(text_widget, text_widget)
        return frame, text_widget

    def make_checklist(parent: Any, height: int = 170) -> Dict[str, Any]:
        frame = ttk.Frame(parent)
        canvas = tk.Canvas(frame, highlightthickness=0, background=WHITE, borderwidth=1, relief="sunken", height=height)
        scroll = make_vscroll(frame, canvas.yview)
        inner = ttk.Frame(canvas)
        window_id = canvas.create_window((0, 0), window=inner, anchor="nw")
        inner.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda e: canvas.itemconfigure(window_id, width=e.width))
        canvas.configure(yscrollcommand=scroll.set)
        canvas.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        _mark_mousewheel(frame, canvas)
        _mark_mousewheel(canvas, canvas)
        _mark_mousewheel(inner, canvas)
        items: List[Tuple[Any, str, str]] = []

        def clear() -> None:
            for child in inner.winfo_children():
                child.destroy()
            items.clear()
            canvas.configure(scrollregion=(0, 0, 0, 0))

        def set_items(lines: List[Tuple[str, str, str]]) -> None:
            clear()
            row = 0
            last_title: Optional[str] = None
            for title, line_html, key in lines:
                if title != last_title:
                    if last_title is not None:
                        row += 1
                    ttk.Label(inner, text=f"{title}", font=("Segoe UI", 10, "bold")).grid(row=row, column=0, sticky="w", padx=6, pady=(5, 2))
                    last_title = title
                    row += 1
                var = tk.BooleanVar(value=True)
                label = _strip_html_for_console(line_html)
                cb = ttk.Checkbutton(inner, text=label, variable=var)
                cb.grid(row=row, column=0, sticky="w", padx=22, pady=1)
                _mark_mousewheel(cb, canvas)
                items.append((var, key, label))
                row += 1
            if not lines:
                ttk.Label(inner, text="No preview lines available.").grid(row=0, column=0, sticky="w", padx=6, pady=6)
            inner.update_idletasks()
            canvas.configure(scrollregion=canvas.bbox("all"))

        def removed_keys() -> List[str]:
            keys: List[str] = []
            for var, key, _label in items:
                if not bool(var.get()) and key not in keys:
                    keys.append(key)
            return keys

        def has_items() -> bool:
            return bool(items)

        return {"frame": frame, "set_items": set_items, "removed_keys": removed_keys, "clear": clear, "has_items": has_items}

    def tile_display_name(key: str) -> str:
        provider = TILE_PROVIDERS.get(str(key), TILE_PROVIDERS[DEFAULT_TILE_KEY])
        return f"{key}) {provider['name']}"

    tile_display_to_key = {tile_display_name(k): k for k in ALL_TILE_KEYS}

    def build_privacy_config_from_vars(mode_var: Any, blanket_var: Any, start_var: Any, end_var: Any, show_var: Any) -> Dict[str, Any]:
        mode = str(mode_var.get() or "off").lower()
        if mode.startswith("off"):
            return {"enabled": False, "meters": 0.0, "start_meters": 0.0, "end_meters": 0.0, "show_status": False}
        try:
            if mode.startswith("default"):
                start_m = end_m = DEFAULT_PRIVACY_METERS
            elif mode.startswith("custom blanket"):
                start_m = end_m = max(0.0, float(blanket_var.get() or 0))
            else:
                start_m = max(0.0, float(start_var.get() or 0))
                end_m = max(0.0, float(end_var.get() or 0))
        except Exception:
            raise ValueError("Privacy distances must be numbers, like 0, 50, 100, or 1000.")
        if start_m <= 0 and end_m <= 0:
            return {"enabled": False, "meters": 0.0, "start_meters": 0.0, "end_meters": 0.0, "show_status": False}
        return {"enabled": True, "meters": max(start_m, end_m), "start_meters": start_m, "end_meters": end_m, "show_status": bool(show_var.get())}

    def create_privacy_controls(parent: Any, row: int, title: str = "Privacy mode") -> Dict[str, Any]:
        lf = ttk.LabelFrame(parent, text=title)
        lf.grid(row=row, column=0, columnspan=4, sticky="ew", padx=6, pady=6)
        for c in range(4):
            lf.columnconfigure(c, weight=1)
        mode_var = tk.StringVar(value="Off")
        blanket_var = tk.StringVar(value=str(int(DEFAULT_PRIVACY_METERS)))
        start_var = tk.StringVar(value="0")
        end_var = tk.StringVar(value="0")
        show_var = tk.BooleanVar(value=False)

        ttk.Label(lf, text="Mode").grid(row=0, column=0, sticky="w", padx=4, pady=2)
        mode_combo = make_combobox(lf, textvariable=mode_var, state="readonly", values=[
            "Off",
            f"Default blanket ({DEFAULT_PRIVACY_METERS:.0f} m start/end)",
            "Custom blanket",
            "Separate start/end",
        ])
        mode_combo.grid(row=0, column=1, columnspan=3, sticky="ew", padx=4, pady=2)

        blanket_widgets: List[Any] = []
        start_end_widgets: List[Any] = []
        status_widgets: List[Any] = []
        blanket_widgets.append(ttk.Label(lf, text="Blanket metres"))
        blanket_widgets[-1].grid(row=1, column=0, sticky="w", padx=4, pady=2)
        blanket_entry = ttk.Entry(lf, textvariable=blanket_var, width=12)
        blanket_entry.grid(row=1, column=1, sticky="w", padx=4, pady=2)
        blanket_widgets.append(blanket_entry)

        start_end_widgets.append(ttk.Label(lf, text="Start metres"))
        start_end_widgets[-1].grid(row=1, column=0, sticky="w", padx=4, pady=2)
        start_entry = ttk.Entry(lf, textvariable=start_var, width=12)
        start_entry.grid(row=1, column=1, sticky="w", padx=4, pady=2)
        start_end_widgets.append(start_entry)
        start_end_widgets.append(ttk.Label(lf, text="End metres"))
        start_end_widgets[-1].grid(row=1, column=2, sticky="w", padx=4, pady=2)
        end_entry = ttk.Entry(lf, textvariable=end_var, width=12)
        end_entry.grid(row=1, column=3, sticky="w", padx=4, pady=2)
        start_end_widgets.append(end_entry)

        status_cb = ttk.Checkbutton(lf, text="Show privacy status line in stats box", variable=show_var)
        status_cb.grid(row=2, column=0, columnspan=4, sticky="w", padx=4, pady=2)
        status_widgets.append(status_cb)

        def update_privacy_visibility(*_args: Any) -> None:
            mode = mode_var.get().lower()
            for w in blanket_widgets + start_end_widgets + status_widgets:
                w.grid_remove()
            if mode.startswith("custom blanket"):
                for w in blanket_widgets:
                    w.grid()
                for w in status_widgets:
                    w.grid()
            elif mode.startswith("separate"):
                for w in start_end_widgets:
                    w.grid()
                for w in status_widgets:
                    w.grid()
            elif mode.startswith("default"):
                for w in status_widgets:
                    w.grid()

        mode_var.trace_add("write", update_privacy_visibility)
        update_privacy_visibility()
        return {"frame": lf, "mode": mode_var, "blanket": blanket_var, "start": start_var, "end": end_var, "show": show_var, "update": update_privacy_visibility}

    def _normalize_entry_path(value: str) -> str:
        return normalize_path(value or "")

    # ------------------------------------------------------------------
    # Shared map options panel
    # ------------------------------------------------------------------
    class MapOptionsPanel:
        """Shared full-option panel used by both Single CSV and Recursive CSV tabs."""
        def __init__(self, parent: Any, get_sample_paths: Any, label: str):
            self.parent = parent
            self.get_sample_paths = get_sample_paths
            self.saved_presets: List[Dict[str, Any]] = []
            self.saved_preset_names: List[str] = []

            self.frame = ttk.LabelFrame(parent, text=label)
            self.frame.pack(fill="x", expand=False, padx=6, pady=6)
            self.frame.columnconfigure(0, weight=1)
            self.frame.columnconfigure(1, weight=1)

            self.color_var = tk.StringVar(value="")
            self.preset_mode_var = tk.StringVar(value="built-in")
            self.saved_preset_var = tk.StringVar(value="")
            self.preset_name_var = tk.StringVar(value="")
            self.map_mode_var = tk.StringVar(value="layers")
            self.initial_tile_var = tk.StringVar(value=tile_display_name(BUILTIN_PRESET_INITIAL_TILE_KEY))
            self.stats_enabled_var = tk.BooleanVar(value=True)
            self.stats_pos_var = tk.StringVar(value="topright")
            self.throttle_var = tk.StringVar(value="CH3(us)")
            self.remove_ardupilot_throttle_var = tk.BooleanVar(value=True)
            self.include_four_sat_var = tk.BooleanVar(value=False)

            self._refresh_saved_preset_names()

            preset_box = ttk.LabelFrame(self.frame, text="Preset and track colour")
            preset_box.grid(row=0, column=0, columnspan=2, sticky="ew", padx=4, pady=4)
            preset_box.columnconfigure(1, weight=1)

            ttk.Label(preset_box, text="Track colour").grid(row=0, column=0, sticky="w", padx=4, pady=2)
            ttk.Entry(preset_box, textvariable=self.color_var).grid(row=0, column=1, sticky="ew", padx=4, pady=2)
            ttk.Label(preset_box, text="ROYGBIV letter/name or #hex; blank = blue").grid(row=0, column=2, sticky="w", padx=4, pady=2)

            preset_choice_frame = ttk.Frame(preset_box)
            preset_choice_frame.grid(row=1, column=0, columnspan=3, sticky="w", padx=0, pady=2)
            ttk.Radiobutton(preset_choice_frame, text="Built-in preset", variable=self.preset_mode_var, value="built-in").pack(side="left", padx=(4, 14))
            ttk.Radiobutton(preset_choice_frame, text="Saved preset", variable=self.preset_mode_var, value="saved").pack(side="left", padx=(0, 14))
            ttk.Radiobutton(preset_choice_frame, text="Customize below", variable=self.preset_mode_var, value="custom").pack(side="left", padx=(0, 4))
            make_wrapped_label(preset_box, "Built-in = one switchable-layer HTML, all stats, top-right stats box, privacy off, OpenStreetMap Standard opens first.").grid(row=2, column=0, columnspan=3, sticky="ew", padx=4, pady=(2, 4))

            self.saved_label = ttk.Label(preset_box, text="Saved preset")
            self.saved_label.grid(row=3, column=0, sticky="w", padx=4, pady=2)
            self.saved_preset_combo = make_combobox(preset_box, textvariable=self.saved_preset_var, values=self.saved_preset_names, state="readonly")
            self.saved_preset_combo.grid(row=3, column=1, columnspan=2, sticky="ew", padx=4, pady=2)

            add_shared_terrain_controls(self.frame, row=1, columnspan=2)

            self.custom_container = ttk.Frame(self.frame)
            self.custom_container.grid(row=2, column=0, columnspan=2, sticky="nsew")
            self.custom_container.columnconfigure(0, weight=1)
            self.custom_container.columnconfigure(1, weight=1)

            # Map/basemap custom section
            self.map_box = ttk.LabelFrame(self.custom_container, text="Basemap output")
            self.map_box.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
            self.map_box.columnconfigure(0, weight=1)
            ttk.Radiobutton(self.map_box, text="One HTML with switchable layers", variable=self.map_mode_var, value="layers").grid(row=0, column=0, sticky="w", padx=4, pady=2)
            ttk.Radiobutton(self.map_box, text="Separate HTML file(s) for selected basemaps", variable=self.map_mode_var, value="separate").grid(row=1, column=0, sticky="w", padx=4, pady=2)

            self.layer_frame = ttk.Frame(self.map_box)
            self.layer_frame.grid(row=2, column=0, sticky="ew", padx=0, pady=(6, 2))
            self.layer_frame.columnconfigure(1, weight=1)
            ttk.Label(self.layer_frame, text="Opening/default layer").grid(row=0, column=0, sticky="w", padx=4, pady=2)
            make_combobox(self.layer_frame, textvariable=self.initial_tile_var, values=[tile_display_name(k) for k in ALL_TILE_KEYS], state="readonly").grid(row=0, column=1, sticky="ew", padx=4, pady=2)

            self.separate_frame = ttk.LabelFrame(self.map_box, text="Basemaps for separate HTML mode")
            self.separate_frame.grid(row=3, column=0, sticky="ew", padx=4, pady=6)
            self.tile_vars: Dict[str, Any] = {}
            for i, key in enumerate(ALL_TILE_KEYS):
                var = tk.BooleanVar(value=(key == DEFAULT_TILE_KEY))
                self.tile_vars[key] = var
                ttk.Checkbutton(self.separate_frame, text=tile_display_name(key), variable=var).grid(row=i, column=0, sticky="w", padx=8, pady=1)

            # Stats custom section
            self.stats_box = ttk.LabelFrame(self.custom_container, text="Flight stats box")
            self.stats_box.grid(row=0, column=1, sticky="nsew", padx=4, pady=4)
            self.stats_box.columnconfigure(1, weight=1)
            ttk.Checkbutton(self.stats_box, text="Add flight stats box", variable=self.stats_enabled_var).grid(row=0, column=0, columnspan=2, sticky="w", padx=4, pady=2)

            self.stat_groups_frame = ttk.Frame(self.stats_box)
            self.stat_groups_frame.grid(row=1, column=0, columnspan=2, sticky="ew")
            self.stat_group_vars: Dict[str, Any] = {}
            for i, num in enumerate(sorted(STATS_GROUPS.keys(), key=int)):
                group_key = STATS_GROUPS[num]["key"]
                var = tk.BooleanVar(value=True)
                self.stat_group_vars[group_key] = var
                ttk.Checkbutton(self.stat_groups_frame, text=STATS_GROUPS[num]["name"], variable=var).grid(row=i, column=0, sticky="w", padx=12, pady=1)

            self.stats_details_frame = ttk.Frame(self.stats_box)
            self.stats_details_frame.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(6, 0))
            self.stats_details_frame.columnconfigure(1, weight=1)
            ttk.Label(self.stats_details_frame, text="Stats box location").grid(row=0, column=0, sticky="w", padx=4, pady=2)
            make_combobox(self.stats_details_frame, textvariable=self.stats_pos_var, state="readonly", values=["topright", "topleft"]).grid(row=0, column=1, sticky="ew", padx=4, pady=2)
            ttk.Label(self.stats_details_frame, text="Throttle channel").grid(row=1, column=0, sticky="w", padx=4, pady=2)
            self.throttle_combo = make_combobox(self.stats_details_frame, textvariable=self.throttle_var, values=["CH3(us)"], state="readonly")
            self.throttle_combo.grid(row=1, column=1, sticky="ew", padx=4, pady=2)
            ttk.Checkbutton(self.stats_details_frame, text="Remove throttle stats from semi-autonomous/controller-managed logs", variable=self.remove_ardupilot_throttle_var).grid(row=2, column=0, columnspan=2, sticky="w", padx=4, pady=2)

            self.gps_rule_box = ttk.LabelFrame(self.custom_container, text="GPS track threshold")
            self.gps_rule_box.grid(row=1, column=0, columnspan=2, sticky="ew", padx=4, pady=4)
            self.gps_rule_box.columnconfigure(0, weight=1)
            ttk.Checkbutton(
                self.gps_rule_box,
                text="Include track rows with exactly 4 satellites (5 satellites remains the recommended default)",
                variable=self.include_four_sat_var,
            ).grid(row=0, column=0, sticky="w", padx=6, pady=2)
            make_wrapped_label(
                self.gps_rule_box,
                "Four-satellite sections may preserve an otherwise continuous route, but position can be less accurate and GPS altitude can freeze or jump. The output/status box and GPS-quality stats will warn when 4-satellite rows were actually included.",
            ).grid(row=1, column=0, sticky="ew", padx=6, pady=(0, 4))

            self.privacy = create_privacy_controls(self.custom_container, 2)

            self.preview_box = ttk.LabelFrame(self.custom_container, text="Stats line preview/removal")
            self.preview_box.grid(row=3, column=0, columnspan=2, sticky="nsew", padx=4, pady=4)
            self.preview_box.columnconfigure(0, weight=1)
            self.preview_box.rowconfigure(0, weight=1)
            self.preview_checklist = make_checklist(self.preview_box, height=190)
            self.preview_checklist["frame"].grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
            make_wrapped_label(self.preview_box, "Click Preview/update, then uncheck any stats lines you do NOT want in the HTML output.").grid(row=1, column=0, sticky="ew", padx=4, pady=2)
            ttk.Button(self.preview_box, text="Preview/update stats line checklist", command=self.preview_stats_lines).grid(row=1, column=1, sticky="e", padx=4, pady=2)

            self.save_box = ttk.LabelFrame(self.custom_container, text="Save preset")
            self.save_box.grid(row=4, column=0, columnspan=2, sticky="ew", padx=4, pady=4)
            self.save_box.columnconfigure(1, weight=1)
            ttk.Label(self.save_box, text="Preset name").grid(row=0, column=0, sticky="w", padx=4, pady=2)
            ttk.Entry(self.save_box, textvariable=self.preset_name_var).grid(row=0, column=1, sticky="ew", padx=4, pady=2)
            ttk.Button(self.save_box, text="Save / overwrite preset", command=self.save_or_overwrite_preset).grid(row=0, column=2, sticky="ew", padx=4, pady=2)

            self.preset_mode_var.trace_add("write", lambda *_: self.update_visibility())
            self.map_mode_var.trace_add("write", lambda *_: self.update_visibility())
            self.stats_enabled_var.trace_add("write", lambda *_: self.update_visibility())
            self.refresh_presets(silent=True)
            self.update_visibility()

        def _refresh_saved_preset_names(self) -> None:
            self.saved_presets = load_user_presets()
            self.saved_preset_names = [p["name"] for p in self.saved_presets]

        def refresh_presets(self, silent: bool = False) -> None:
            self._refresh_saved_preset_names()
            try:
                self.saved_preset_combo.configure(values=self.saved_preset_names)
                if self.saved_preset_names and not self.saved_preset_var.get():
                    self.saved_preset_var.set(self.saved_preset_names[0])
            except Exception:
                pass
            if not silent:
                log("Preset list refreshed.")

        def update_visibility(self) -> None:
            mode = self.preset_mode_var.get()
            if mode == "saved":
                self.saved_label.grid()
                self.saved_preset_combo.grid()
            else:
                self.saved_label.grid_remove()
                self.saved_preset_combo.grid_remove()

            if mode == "custom":
                self.custom_container.grid()
            else:
                self.custom_container.grid_remove()
                return

            if self.map_mode_var.get() == "layers":
                self.layer_frame.grid()
                self.separate_frame.grid_remove()
            else:
                self.layer_frame.grid_remove()
                self.separate_frame.grid()

            if bool(self.stats_enabled_var.get()):
                self.stat_groups_frame.grid()
                self.stats_details_frame.grid()
                self.preview_box.grid()
            else:
                self.stat_groups_frame.grid_remove()
                self.stats_details_frame.grid_remove()
                self.preview_box.grid_remove()

            try:
                scan = self.frame
                while scan is not None:
                    updater = getattr(scan, "_fpv_update_scrollregion", None)
                    if updater is not None:
                        root.after_idle(updater)
                        break
                    scan = getattr(scan, "master", None)
            except Exception:
                pass

        def sample_paths(self) -> List[str]:
            paths: List[str] = []
            for p in self.get_sample_paths():
                if p:
                    paths.append(normalize_path(p))
            return paths

        def refresh_throttle_channels(self, silent: bool = True) -> None:
            paths = [p for p in self.sample_paths() if os.path.isfile(p)]
            header = get_csv_header(paths[0]) if paths else []
            channels = available_channel_columns_from_header(header)
            if not channels:
                channels = ["CH3(us)"]
            self.throttle_combo.configure(values=channels)
            if self.throttle_var.get() not in channels:
                if "CH3(us)" in channels:
                    self.throttle_var.set("CH3(us)")
                else:
                    self.throttle_var.set(channels[0])
            if not silent:
                log(f"Throttle channel choices refreshed: {', '.join(channels)}")

        def _tile_key_from_display(self, value: str) -> str:
            return tile_display_to_key.get(value, DEFAULT_TILE_KEY)

        def _privacy_config(self) -> Dict[str, Any]:
            return build_privacy_config_from_vars(self.privacy["mode"], self.privacy["blanket"], self.privacy["start"], self.privacy["end"], self.privacy["show"])

        def _custom_stats_config(self) -> Dict[str, Any]:
            if not bool(self.stats_enabled_var.get()):
                return {"enabled": False, "groups": [], "position": self.stats_pos_var.get() or "topright"}
            groups = [key for key, var in self.stat_group_vars.items() if bool(var.get())]
            if not groups:
                raise ValueError("Select at least one stats group, or turn off the stats box.")
            return {
                "enabled": True,
                "groups": groups,
                "position": self.stats_pos_var.get() or "topright",
                "throttle_channel": self.throttle_var.get() or "CH3(us)",
                "remove_throttle_for_ardupilot_logs": bool(self.remove_ardupilot_throttle_var.get()),
            }

        def _selected_tile_keys(self) -> List[str]:
            keys = [k for k, var in self.tile_vars.items() if bool(var.get())]
            if not keys:
                raise ValueError("Select at least one basemap for separate-file mode.")
            return keys

        def _base_run_options(self, color_css: str) -> Dict[str, Any]:
            paths = self.sample_paths()
            sample = paths[0] if paths else None
            mode = self.preset_mode_var.get()
            if mode == "built-in":
                opts = default_preset_options(color_css, sample)
                opts["stats_config"]["remove_throttle_for_ardupilot_logs"] = True
                return opts
            if mode == "saved":
                preset_name = self.saved_preset_var.get()
                preset = next((p for p in self.saved_presets if p["name"] == preset_name), None)
                if preset is None:
                    raise ValueError("Choose a saved preset or switch to built-in/custom.")
                return _sanitize_loaded_run_options(preset.get("options", {}), color_css)

            initial_key = self._tile_key_from_display(self.initial_tile_var.get())
            map_mode = self.map_mode_var.get()
            tile_keys = _ordered_tile_keys(initial_key, ALL_TILE_KEYS) if map_mode == "layers" else self._selected_tile_keys()
            return {
                "color_css": color_css,
                "map_mode": map_mode,
                "tile_keys": tile_keys,
                "initial_tile_key": initial_key,
                "stats_config": self._custom_stats_config(),
                "privacy_config": self._privacy_config(),
                "min_sats": RELAXED_MIN_SATS if bool(self.include_four_sat_var.get()) else MIN_SATS,
            }

        def build_run_options(self, files: List[str]) -> Dict[str, Any]:
            color = parse_color(self.color_var.get())
            if color is None:
                raise ValueError("Colour not recognized. Use ROYGBIV letters/words like b/blue/y/yellow, or #00aaff.")
            opts = self._base_run_options(color)
            removed_keys = self.preview_checklist["removed_keys"]() if self.preview_checklist["has_items"]() else []
            if removed_keys:
                stats_config = dict(opts.get("stats_config", {}))
                stats_config["removed_line_keys"] = removed_keys
                stats_config.pop("removed_line_indices", None)
                opts["stats_config"] = stats_config
            return opts

        def _build_preview_lines(self) -> List[Tuple[str, str, str]]:
            paths = [p for p in self.sample_paths() if os.path.isfile(p)]
            if not paths:
                raise ValueError("Choose a CSV first so stats lines can be previewed.")
            color = parse_color(self.color_var.get()) or "#3388ff"
            opts = self._base_run_options(color)
            stats_config = opts.get("stats_config", {})
            if not stats_config.get("enabled"):
                raise ValueError("Stats box is off, so there are no stats lines to preview.")
            unique_structures: List[Tuple[str, List[str]]] = []
            seen = set()
            for path in paths:
                effective = effective_stats_config_for_csv(stats_config, path)
                throttle_col = effective.get("throttle_channel", "CH3(us)")
                data = read_flight_data(path, min_sats=int(opts.get("min_sats", MIN_SATS)), throttle_col_name=throttle_col)
                preview_stats = data.get("flight_stats", {})
                if "agl_altitude" in effective.get("groups", []):
                    preview_stats, _preview_warnings = _augment_flight_stats_with_agl(path, preview_stats)
                lines = _flight_stat_lines(preview_stats, effective.get("groups", ["default"]), opts.get("privacy_config", {"enabled": False}), throttle_col)
                key_tuple = tuple(_stat_line_keys(lines))
                if key_tuple not in seen:
                    seen.add(key_tuple)
                    unique_structures.append((os.path.basename(path), lines))
                if len(unique_structures) >= 20:
                    break
            flattened: List[Tuple[str, str, str]] = []
            for title, lines in unique_structures:
                for line, key in zip(lines, _stat_line_keys(lines)):
                    flattened.append((title, line, key))
            return flattened

        def preview_stats_lines(self) -> None:
            try:
                self.preview_checklist["set_items"](self._build_preview_lines())
            except Exception as exc:
                messagebox.showerror("Stats preview failed", str(exc))

        def _options_for_preset_storage_from_gui(self) -> Dict[str, Any]:
            color = parse_color(self.color_var.get()) or "#3388ff"
            opts = self._base_run_options(color)
            return _run_options_for_preset_storage(opts)

        def save_or_overwrite_preset(self) -> None:
            name = self.preset_name_var.get().strip()
            if not name:
                if self.preset_mode_var.get() == "saved" and self.saved_preset_var.get():
                    name = self.saved_preset_var.get().strip()
                else:
                    messagebox.showwarning("Preset name needed", "Type a preset name first, or choose a saved preset to overwrite.")
                    return
            try:
                stored = self._options_for_preset_storage_from_gui()
                presets = load_user_presets()
                for i, preset in enumerate(presets):
                    if preset["name"].strip().lower() == name.lower():
                        if not messagebox.askyesno("Overwrite preset?", f"Preset '{preset['name']}' already exists. Overwrite it?"):
                            return
                        presets[i] = {"name": preset["name"], "options": stored}
                        save_user_presets(presets)
                        self.refresh_presets(silent=True)
                        self.saved_preset_var.set(preset["name"])
                        log(f"Overwrote preset: {preset['name']}")
                        return
                presets.append({"name": name, "options": stored})
                save_user_presets(presets)
                self.refresh_presets(silent=True)
                self.saved_preset_var.set(name)
                log(f"Saved preset: {name}")
            except Exception as exc:
                messagebox.showerror("Preset save failed", str(exc))

    # ------------------------------------------------------------------
    # Main window
    # ------------------------------------------------------------------
    root.columnconfigure(0, weight=1)
    root.rowconfigure(0, weight=4)
    root.rowconfigure(1, weight=1, minsize=145)

    notebook = ttk.Notebook(root)
    notebook.grid(row=0, column=0, sticky="nsew", padx=10, pady=(10, 4))

    log_frame = ttk.Frame(root)
    log_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=(4, 10))
    log_frame.columnconfigure(0, weight=1)
    log_frame.rowconfigure(1, weight=1)
    ttk.Label(log_frame, text="Output / status").grid(row=0, column=0, sticky="w")
    log_container, log_text = make_scrolled_text(log_frame, height=6, wrap="word")
    log_container.grid(row=1, column=0, sticky="nsew")

    # Shared terrain setting displayed in every operational mode. All controls use the
    # same variables and save to the existing JSON, so changing one tab updates all others.
    _terrain_saved = load_parameter_settings()
    shared_terrain_source_var = tk.StringVar(value=str(_terrain_saved.get("terrain_source", "Local terrain files")))
    shared_terrain_folder_var = tk.StringVar(value=str(_terrain_saved.get("terrain_folder", "") or ""))
    _terrain_save_after: Optional[str] = None

    def save_shared_terrain_settings(*_args: Any) -> None:
        nonlocal _terrain_save_after
        if _terrain_save_after:
            try: root.after_cancel(_terrain_save_after)
            except Exception: pass
        def do_save() -> None:
            settings = load_parameter_settings()
            settings["terrain_source"] = shared_terrain_source_var.get()
            settings["terrain_folder"] = shared_terrain_folder_var.get().strip()
            save_parameter_settings(settings)
        _terrain_save_after = root.after(350, do_save)

    shared_terrain_source_var.trace_add("write", save_shared_terrain_settings)
    shared_terrain_folder_var.trace_add("write", save_shared_terrain_settings)

    def add_shared_terrain_controls(parent: Any, row: Optional[int] = None, columnspan: int = 3, compact: bool = False) -> Any:
        box = ttk.LabelFrame(parent, text="Terrain data source (shared across all modes)")
        if row is None:
            box.pack(fill="x", padx=6, pady=6)
        else:
            box.grid(row=row, column=0, columnspan=columnspan, sticky="ew", padx=8, pady=6)
        box.columnconfigure(1, weight=1)
        ttk.Label(box, text="Source").grid(row=0, column=0, sticky="w", padx=4, pady=2)
        combo = make_combobox(box, textvariable=shared_terrain_source_var, values=["Local terrain files", "OpenTopoData online", "Local first, then online fallback"], state="readonly")
        combo.grid(row=0, column=1, columnspan=2, sticky="ew", padx=4, pady=2)
        folder_label = ttk.Label(box, text="Terrain folder")
        folder_label.grid(row=1, column=0, sticky="w", padx=4, pady=2)
        folder_entry = ttk.Entry(box, textvariable=shared_terrain_folder_var)
        folder_entry.grid(row=1, column=1, sticky="ew", padx=4, pady=2)
        def browse() -> None:
            chosen = filedialog.askdirectory(title="Choose folder containing ArduPilot DAT or SRTM HGT terrain files", initialdir=shared_terrain_folder_var.get() or None)
            if chosen: shared_terrain_folder_var.set(chosen)
        folder_button = ttk.Button(box, text="Browse", command=browse)
        folder_button.grid(row=1, column=2, sticky="e", padx=4, pady=2)
        def update(*_a: Any) -> None:
            if shared_terrain_source_var.get().startswith("OpenTopoData"):
                folder_label.grid_remove(); folder_entry.grid_remove(); folder_button.grid_remove()
            else:
                folder_label.grid(); folder_entry.grid(); folder_button.grid()
        shared_terrain_source_var.trace_add("write", update); update()
        return box

    def terrain_preflight(operation_name: str, needs_terrain: bool) -> bool:
        if not needs_terrain:
            return True
        source = shared_terrain_source_var.get()
        folder = shared_terrain_folder_var.get().strip()
        if source.startswith("Local") and (not folder or not os.path.isdir(folder)):
            text = (f"{operation_name} needs terrain data, but no valid local terrain folder is selected.\n\n"
                    "Choose Cancel to fix the setting, or Continue to let the operation use its normal fallbacks where available.")
            return bool(messagebox.askokcancel("Terrain data not ready", text, icon="warning", parent=root))
        if source.startswith("Local"):
            try:
                _get_local_terrain_database(folder)
            except Exception as exc:
                text = (f"{operation_name} could not build a recursive terrain index from the selected folder.\n\n{exc}\n\n"
                        "Choose Cancel to fix the folder/files, or Continue to let the operation use its normal fallbacks where available.")
                return bool(messagebox.askokcancel("Terrain files not ready", text, icon="warning", parent=root))
        return True

    def operation_start(title: str) -> None:
        log("")
        log(title)

    def show_warning_summary(title: str, warnings: List[str], recursive: bool = False) -> None:
        clean = []
        for item in warnings:
            text = str(item).strip()
            if text and text not in clean: clean.append(text)
        if not clean: return
        if recursive:
            messagebox.showwarning(title, f"The batch finished with {len(clean)} warning/error type(s). See the Output / status box for details.", parent=root)
        else:
            messagebox.showwarning(title, "The file was produced, but an important warning occurred:\n\n" + "\n".join(clean[:8]), parent=root)

    def important_warnings_from_text(text: str) -> List[str]:
        warnings: List[str] = []
        important_terms = ("⚠", "warning:", "failed", "no local terrain folder", "no terrain", "terrain unavailable", "returned no elevations", "coverage was incomplete", "could not be calculated", "no usable gps")
        for line in str(text or "").splitlines():
            clean = line.strip().replace("⚠️", "").replace("⚠", "").strip()
            low = clean.lower()
            if clean and any(term in low for term in important_terms):
                if clean not in warnings: warnings.append(clean)
        return warnings

    initial_paths = [normalize_path(p) for p in sys.argv[1:] if not str(p).startswith("-")]
    initial_csvs = [p for p in initial_paths if os.path.isfile(p) and p.lower().endswith(".csv")]
    initial_folders = [p for p in initial_paths if os.path.isdir(p)]

    # ------------------------------------------------------------------
    # Tab 1: single CSV map
    # ------------------------------------------------------------------
    single_tab = ttk.Frame(notebook)
    notebook.add(single_tab, text="Process single CSV")
    single_tab.columnconfigure(1, weight=1)
    single_tab.rowconfigure(3, weight=1)

    ttk.Label(single_tab, text="Process a single CSV into an interactive HTML map", font=("Segoe UI", 12, "bold")).grid(row=0, column=0, columnspan=3, sticky="w", padx=8, pady=(8, 4))
    make_wrapped_label(single_tab, "Choose a CSV with Browse or paste a file path. Windows Copy as path text with quotes is accepted.").grid(row=1, column=0, columnspan=3, sticky="ew", padx=8, pady=(0, 4))
    single_path_var = tk.StringVar(value=initial_csvs[0] if initial_csvs else "")
    ttk.Label(single_tab, text="CSV file").grid(row=2, column=0, sticky="w", padx=8, pady=4)
    single_entry = ttk.Entry(single_tab, textvariable=single_path_var)
    single_entry.grid(row=2, column=1, sticky="ew", padx=6, pady=4)
    add_dnd_to_entry(single_entry, single_path_var)

    def browse_single_csv() -> None:
        p = filedialog.askopenfilename(title="Choose CSV file", filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
        if p:
            single_path_var.set(p)

    ttk.Button(single_tab, text="Browse", command=browse_single_csv).grid(row=2, column=2, sticky="ew", padx=8, pady=4)
    single_scroll, single_inner, _single_canvas = make_scrolled_frame(single_tab)
    single_scroll.grid(row=3, column=0, columnspan=3, sticky="nsew", padx=4, pady=4)
    single_options = MapOptionsPanel(single_inner, lambda: [single_path_var.get()], "Single CSV map options")

    _single_after_id: Optional[str] = None
    def _single_path_changed(*_args: Any) -> None:
        nonlocal _single_after_id
        if _single_after_id:
            try: root.after_cancel(_single_after_id)
            except Exception: pass
        _single_after_id = root.after(400, lambda: single_options.refresh_throttle_channels(silent=True))
    single_path_var.trace_add("write", _single_path_changed)

    def generate_single_map() -> None:
        try:
            csv_path = normalize_path(single_path_var.get())
            if not os.path.isfile(csv_path):
                raise ValueError("Choose a valid CSV file. Pasted Copy as path values with quotes are okay.")
            run_options = single_options.build_run_options([csv_path])
            needs_terrain = "agl_altitude" in run_options.get("stats_config", {}).get("groups", [])
            if not terrain_preflight("Single CSV map export", needs_terrain): return
            start_index = log_text.index("end-1c")
            old_stdout = sys.stdout
            sys.stdout = _TextRedirector(log_text)
            try:
                operation_start(f"Single CSV export: {csv_path}")
                made = process_csv_to_html(csv_path, run_options)
                print(f"Done. Created {made} HTML map file(s).")
            finally:
                sys.stdout = old_stdout
            show_warning_summary("Single CSV export warning", important_warnings_from_text(log_text.get(start_index, "end-1c")), recursive=False)
        except Exception as exc:
            messagebox.showerror("Single CSV export failed", str(exc))

    single_buttons = ttk.Frame(single_tab)
    single_buttons.grid(row=4, column=0, columnspan=3, sticky="ew", padx=8, pady=6)
    ttk.Button(single_buttons, text="Generate HTML map", command=generate_single_map).pack(side="right", padx=(8, 0))

    # ------------------------------------------------------------------
    # Tab 2: recursive CSV maps
    # ------------------------------------------------------------------
    recursive_tab = ttk.Frame(notebook)
    notebook.add(recursive_tab, text="Process CSVs recursively")
    recursive_tab.columnconfigure(1, weight=1)
    recursive_tab.rowconfigure(3, weight=1)

    ttk.Label(recursive_tab, text="Process all CSV files recursively in a folder", font=("Segoe UI", 12, "bold")).grid(row=0, column=0, columnspan=3, sticky="w", padx=8, pady=(8, 4))
    make_wrapped_label(recursive_tab, "Choose a folder, paste a folder path, or paste Windows Copy as path text with quotes.").grid(row=1, column=0, columnspan=3, sticky="ew", padx=8, pady=(0, 4))
    recursive_path_var = tk.StringVar(value=initial_folders[0] if initial_folders else "")
    ttk.Label(recursive_tab, text="Folder").grid(row=2, column=0, sticky="w", padx=8, pady=4)
    recursive_entry = ttk.Entry(recursive_tab, textvariable=recursive_path_var)
    recursive_entry.grid(row=2, column=1, sticky="ew", padx=6, pady=4)
    add_dnd_to_entry(recursive_entry, recursive_path_var)

    def browse_recursive_folder() -> None:
        p = filedialog.askdirectory(title="Choose folder")
        if p:
            recursive_path_var.set(p)

    ttk.Button(recursive_tab, text="Browse", command=browse_recursive_folder).grid(row=2, column=2, sticky="ew", padx=8, pady=4)
    recursive_scroll, recursive_inner, _recursive_canvas = make_scrolled_frame(recursive_tab)
    recursive_scroll.grid(row=3, column=0, columnspan=3, sticky="nsew", padx=4, pady=4)

    def recursive_sample_paths() -> List[str]:
        folder = normalize_path(recursive_path_var.get())
        if os.path.isdir(folder):
            return find_csv_files(folder)
        return []

    recursive_options = MapOptionsPanel(recursive_inner, recursive_sample_paths, "Recursive map options")
    _recursive_after_id: Optional[str] = None
    def _recursive_path_changed(*_args: Any) -> None:
        nonlocal _recursive_after_id
        if _recursive_after_id:
            try: root.after_cancel(_recursive_after_id)
            except Exception: pass
        _recursive_after_id = root.after(600, lambda: recursive_options.refresh_throttle_channels(silent=True))
    recursive_path_var.trace_add("write", _recursive_path_changed)

    def generate_recursive_maps() -> None:
        try:
            folder = normalize_path(recursive_path_var.get())
            if not os.path.isdir(folder):
                raise ValueError("Choose a valid folder. Pasted Copy as path values with quotes are okay.")
            csv_files = find_csv_files(folder)
            if not csv_files:
                raise ValueError("No CSV files found in that folder.")
            run_options = recursive_options.build_run_options(csv_files)
            needs_terrain = "agl_altitude" in run_options.get("stats_config", {}).get("groups", [])
            if not terrain_preflight("Recursive CSV map export", needs_terrain): return
            start_index = log_text.index("end-1c")
            old_stdout = sys.stdout
            sys.stdout = _TextRedirector(log_text)
            try:
                operation_start(f"Recursive export: {len(csv_files)} CSV file(s)")
                made = 0
                for i, csv_path in enumerate(csv_files, start=1):
                    print(f"\n[{i}/{len(csv_files)}] {os.path.basename(csv_path)}")
                    made += process_csv_to_html(csv_path, run_options)
                print(f"\nDone. Created {made} HTML map file(s).")
            finally:
                sys.stdout = old_stdout
            show_warning_summary("Recursive export warnings", important_warnings_from_text(log_text.get(start_index, "end-1c")), recursive=True)
        except Exception as exc:
            messagebox.showerror("Recursive export failed", str(exc))

    recursive_buttons = ttk.Frame(recursive_tab)
    recursive_buttons.grid(row=4, column=0, columnspan=3, sticky="ew", padx=8, pady=6)
    ttk.Button(recursive_buttons, text="Generate HTML maps", command=generate_recursive_maps).pack(side="right", padx=(8, 0))

    # ------------------------------------------------------------------
    # Tab 3: integrated flight data analysis
    # ------------------------------------------------------------------
    analysis_tab = ttk.Frame(notebook)
    notebook.add(analysis_tab, text="Flight data analysis")
    analysis_tab.columnconfigure(0, weight=1)
    analysis_tab.rowconfigure(0, weight=1)
    analysis_outer, analysis_inner, analysis_canvas = make_scrolled_frame(analysis_tab)
    analysis_outer.grid(row=0, column=0, sticky="nsew")
    add_shared_terrain_controls(analysis_inner, row=98, columnspan=4)
    analysis_inner.columnconfigure(1, weight=1)

    ttk.Label(analysis_inner, text="Interactive analysis for any telemetry or computed parameter", font=("Segoe UI", 12, "bold")).grid(row=0, column=0, columnspan=4, sticky="w", padx=8, pady=(8, 4))
    make_wrapped_label(
        analysis_inner,
        "Select the main parameter to analyse. The app builds the same polished map, flagged-episode table, inspection points, deterministic findings, and smooth Plotly timeline for satellite quality, signal, speed, altitude, power, controls, or another available CSV value. The threshold controls change to match the selected parameter.",
    ).grid(row=1, column=0, columnspan=4, sticky="ew", padx=8, pady=(0, 6))

    analysis_path_var = tk.StringVar(value=initial_csvs[0] if initial_csvs else "")
    ttk.Label(analysis_inner, text="CSV file").grid(row=2, column=0, sticky="w", padx=8, pady=4)
    analysis_entry = ttk.Entry(analysis_inner, textvariable=analysis_path_var)
    analysis_entry.grid(row=2, column=1, columnspan=2, sticky="ew", padx=6, pady=4)
    add_dnd_to_entry(analysis_entry, analysis_path_var)
    def browse_analysis_csv() -> None:
        p = filedialog.askopenfilename(filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
        if p: analysis_path_var.set(p)
    ttk.Button(analysis_inner, text="Browse", command=browse_analysis_csv).grid(row=2, column=3, sticky="ew", padx=8, pady=4)

    analysis_metric_options: List[Dict[str, Any]] = []
    analysis_metric_var = tk.StringVar(value="")
    ttk.Label(analysis_inner, text="Main parameter to analyse").grid(row=3, column=0, sticky="w", padx=8, pady=4)
    analysis_metric_combo = make_combobox(analysis_inner, textvariable=analysis_metric_var, state="readonly")
    analysis_metric_combo.grid(row=3, column=1, columnspan=3, sticky="ew", padx=6, pady=4)

    analysis_initial_tile_var = tk.StringVar(value=tile_display_name(BUILTIN_PRESET_INITIAL_TILE_KEY))
    ttk.Label(analysis_inner, text="Opening/default basemap").grid(row=4, column=0, sticky="w", padx=8, pady=4)
    make_combobox(analysis_inner, textvariable=analysis_initial_tile_var, values=[tile_display_name(k) for k in ALL_TILE_KEYS], state="readonly").grid(row=4, column=1, columnspan=3, sticky="ew", padx=6, pady=4)

    parameter_box = ttk.LabelFrame(analysis_inner, text="Fine-tune this parameter's analysis")
    parameter_box.grid(row=5, column=0, columnspan=4, sticky="ew", padx=8, pady=6)
    parameter_box.columnconfigure(1, weight=1)
    analysis_rule_var = tk.StringVar(value="Low / medium / high bands")
    analysis_good_threshold_var = tk.StringVar(value="")
    analysis_bad_threshold_var = tk.StringVar(value="")
    analysis_good_label_var = tk.StringVar(value="Low / good threshold")
    analysis_bad_label_var = tk.StringVar(value="High / bad threshold")
    analysis_observed_var = tk.StringVar(value="Choose a CSV and parameter to see its observed range.")
    ttk.Label(parameter_box, text="Interpretation").grid(row=0, column=0, sticky="w", padx=6, pady=4)
    analysis_rule_combo = make_combobox(parameter_box, textvariable=analysis_rule_var, values=["Higher values are better", "Lower values are better", "Low / medium / high bands"], state="readonly")
    analysis_rule_combo.grid(row=0, column=1, columnspan=3, sticky="ew", padx=6, pady=4)
    analysis_good_label = ttk.Label(parameter_box, textvariable=analysis_good_label_var)
    analysis_good_label.grid(row=1, column=0, sticky="w", padx=6, pady=4)
    ttk.Entry(parameter_box, textvariable=analysis_good_threshold_var, width=16).grid(row=1, column=1, sticky="w", padx=6, pady=4)
    analysis_bad_label = ttk.Label(parameter_box, textvariable=analysis_bad_label_var)
    analysis_bad_label.grid(row=1, column=2, sticky="w", padx=6, pady=4)
    ttk.Entry(parameter_box, textvariable=analysis_bad_threshold_var, width=16).grid(row=1, column=3, sticky="w", padx=6, pady=4)
    make_wrapped_label(parameter_box, "Automatic values are practical starting points. Edit them to match your receiver, aircraft, battery, local rules, or the specific question you are investigating.").grid(row=2, column=0, columnspan=4, sticky="ew", padx=6, pady=(2, 0))
    ttk.Label(parameter_box, textvariable=analysis_observed_var).grid(row=3, column=0, columnspan=4, sticky="w", padx=6, pady=(2, 6))

    timeline_box = ttk.LabelFrame(analysis_inner, text="Supporting timeline data")
    timeline_box.grid(row=6, column=0, columnspan=4, sticky="ew", padx=8, pady=6)
    timeline_box.columnconfigure(0, weight=1)
    make_wrapped_label(timeline_box, "The analysed parameter is always plotted first. Select extra computed quantities or any usable numeric original CSV column for comparison. Date/time values are plotted as a true time axis with a limited number of ticks, preventing the overlapping white labels seen in v25.").grid(row=0, column=0, columnspan=4, sticky="ew", padx=6, pady=4)
    analysis_timeline_vars: Dict[str, Any] = {}
    analysis_timeline_options: List[Dict[str, str]] = []
    timeline_checks_frame, timeline_checks_inner, timeline_checks_canvas = make_scrolled_frame(timeline_box, height=250)
    timeline_checks_frame.grid(row=1, column=0, columnspan=4, sticky="ew", padx=6, pady=4)
    timeline_checks_inner.columnconfigure(0, weight=1)

    def selected_analysis_metric() -> Optional[Dict[str, Any]]:
        label = analysis_metric_var.get()
        return next((o for o in analysis_metric_options if str(o.get("label")) == label), None)

    def update_analysis_rule_labels(*_args: Any) -> None:
        rule = analysis_rule_var.get()
        metric = selected_analysis_metric() or {}
        if str(metric.get("id")) == "satellites":
            analysis_good_label_var.set("Trusted at/above (sats)")
            analysis_bad_label_var.set("Marginal minimum (sats)")
        elif rule.startswith("Higher"):
            analysis_good_label_var.set("Good at/above")
            analysis_bad_label_var.set("Bad below")
        elif rule.startswith("Lower"):
            analysis_good_label_var.set("Good at/below")
            analysis_bad_label_var.set("Bad above")
        else:
            analysis_good_label_var.set("Low/medium boundary")
            analysis_bad_label_var.set("Medium/high boundary")

    def refresh_analysis_parameter_defaults(*_args: Any) -> None:
        metric = selected_analysis_metric()
        path = normalize_path(analysis_path_var.get()) if analysis_path_var.get().strip() else ""
        if not metric or not os.path.isfile(path):
            return
        try:
            defaults = analysis_defaults_for_csv(path, metric)
            saved_settings = load_parameter_settings()
            profiles = saved_settings.get("analysis_profiles", {}) if isinstance(saved_settings.get("analysis_profiles", {}), dict) else {}
            saved_profile = profiles.get(str(metric.get("id")), {}) if isinstance(profiles.get(str(metric.get("id")), {}), dict) else {}
            rule = saved_profile.get("rule", defaults.get("rule", "bands"))
            good_value = saved_profile.get("good_threshold", defaults.get("good_threshold"))
            bad_value = saved_profile.get("bad_threshold", defaults.get("bad_threshold"))
            analysis_rule_var.set("Higher values are better" if rule == "higher" else "Lower values are better" if rule == "lower" else "Low / medium / high bands")
            analysis_good_threshold_var.set(_format_csv_number(good_value, 3))
            analysis_bad_threshold_var.set(_format_csv_number(bad_value, 3))
            unit = str(metric.get("unit", "value"))
            saved_note = " Saved thresholds loaded." if saved_profile else ""
            analysis_observed_var.set(f"Observed: min {_format_num(defaults.get('min'), 2, ' '+unit)}, median {_format_num(defaults.get('median'), 2, ' '+unit)}, max {_format_num(defaults.get('max'), 2, ' '+unit)}. {defaults.get('source_note','')}{saved_note}")
            update_analysis_rule_labels()
        except Exception as exc:
            analysis_observed_var.set(f"Could not calculate automatic thresholds: {exc}")

    def refresh_analysis_metrics(silent: bool = True) -> None:
        nonlocal analysis_timeline_options, analysis_metric_options
        for child in timeline_checks_inner.winfo_children(): child.destroy()
        analysis_timeline_vars.clear()
        path = normalize_path(analysis_path_var.get()) if analysis_path_var.get().strip() else ""
        if not path or not os.path.isfile(path):
            analysis_metric_options=[]; analysis_metric_combo.configure(values=[])
            ttk.Label(timeline_checks_inner, text="Choose a CSV to load parameters and timeline choices.").grid(row=0,column=0,sticky="w",padx=6,pady=6)
            return
        try:
            analysis_metric_options = _analysis_metric_options(path)
            labels=[str(o["label"]) for o in analysis_metric_options]
            analysis_metric_combo.configure(values=labels)
            if labels and analysis_metric_var.get() not in labels:
                preferred=next((str(o["label"]) for o in analysis_metric_options if o.get("id")=="satellites"),labels[0])
                analysis_metric_var.set(preferred)
            analysis_timeline_options=analysis_timeline_options_for_csv(path)
            defaults={"sats","logged_speed","coord_speed","relative_alt"}
            for i,option in enumerate(analysis_timeline_options):
                var=tk.BooleanVar(value=option["id"] in defaults); analysis_timeline_vars[option["id"]]=var
                cb=ttk.Checkbutton(timeline_checks_inner,text=f"{option['label']} ({option['unit']})",variable=var)
                cb.grid(row=i,column=0,sticky="w",padx=8,pady=1); _mark_mousewheel(cb,timeline_checks_canvas)
            timeline_checks_inner.update_idletasks(); timeline_checks_canvas.configure(scrollregion=timeline_checks_canvas.bbox("all"))
            refresh_analysis_parameter_defaults()
            if not silent: log(f"Loaded {len(labels)} analysable parameters and {len(analysis_timeline_options)} supporting timeline choices.")
        except Exception as exc:
            ttk.Label(timeline_checks_inner,text=f"Could not inspect CSV: {exc}").grid(row=0,column=0,sticky="w",padx=6,pady=6)
            if not silent: messagebox.showerror("Analysis CSV inspection failed",str(exc))

    def set_analysis_timeline_defaults() -> None:
        defaults={"sats","logged_speed","coord_speed","relative_alt"}
        for field_id,var in analysis_timeline_vars.items(): var.set(field_id in defaults)
    def set_analysis_timeline_all(value: bool) -> None:
        for var in analysis_timeline_vars.values(): var.set(value)
    timeline_buttons=ttk.Frame(timeline_box); timeline_buttons.grid(row=2,column=0,columnspan=4,sticky="w",padx=6,pady=(0,4))
    ttk.Button(timeline_buttons,text="Recommended",command=set_analysis_timeline_defaults).pack(side="left",padx=(0,6))
    ttk.Button(timeline_buttons,text="Select all",command=lambda:set_analysis_timeline_all(True)).pack(side="left",padx=6)
    ttk.Button(timeline_buttons,text="Clear",command=lambda:set_analysis_timeline_all(False)).pack(side="left",padx=6)

    saved_analysis_export = load_parameter_settings()
    analysis_png_width_var = tk.StringVar(value=str(saved_analysis_export.get("analysis_png_width", 1920)))
    analysis_png_height_var = tk.StringVar(value=str(saved_analysis_export.get("analysis_png_height", 1080)))
    analysis_chart_title_var = tk.StringVar(value=str(saved_analysis_export.get("analysis_chart_title", "") or ""))
    analysis_png_filename_var = tk.StringVar(value=str(saved_analysis_export.get("analysis_png_filename", "") or ""))
    graph_export_box = ttk.LabelFrame(analysis_inner, text="Standalone timeline PNG export")
    graph_export_box.grid(row=7,column=0,columnspan=4,sticky="ew",padx=8,pady=6)
    graph_export_box.columnconfigure(1,weight=1); graph_export_box.columnconfigure(3,weight=1)
    ttk.Label(graph_export_box,text="PNG width (px)").grid(row=0,column=0,sticky="w",padx=6,pady=4)
    ttk.Entry(graph_export_box,textvariable=analysis_png_width_var,width=12).grid(row=0,column=1,sticky="w",padx=6,pady=4)
    ttk.Label(graph_export_box,text="PNG height (px)").grid(row=0,column=2,sticky="w",padx=6,pady=4)
    ttk.Entry(graph_export_box,textvariable=analysis_png_height_var,width=12).grid(row=0,column=3,sticky="w",padx=6,pady=4)
    ttk.Label(graph_export_box,text="Chart title (optional)").grid(row=1,column=0,sticky="w",padx=6,pady=4)
    ttk.Entry(graph_export_box,textvariable=analysis_chart_title_var).grid(row=1,column=1,columnspan=3,sticky="ew",padx=6,pady=4)
    ttk.Label(graph_export_box,text="PNG filename (optional)").grid(row=2,column=0,sticky="w",padx=6,pady=4)
    ttk.Entry(graph_export_box,textvariable=analysis_png_filename_var).grid(row=2,column=1,columnspan=3,sticky="ew",padx=6,pady=4)
    make_wrapped_label(graph_export_box,"The generated HTML includes a dedicated download button and Plotly camera control. Defaults are exact Full HD 1920 × 1080. The browser saves only the graph as a standalone PNG; no extra Python package is needed.").grid(row=3,column=0,columnspan=4,sticky="ew",padx=6,pady=(2,6))

    analysis_privacy=create_privacy_controls(analysis_inner,8,title="Privacy mode for analysis report")
    make_wrapped_label(analysis_inner,"The map retains valid 4-satellite positions for inspection, while GPS below 4 satellites or missing coordinates break the route. Privacy-trimmed coordinates are removed before the HTML is built.").grid(row=9,column=0,columnspan=4,sticky="ew",padx=8,pady=4)

    def generate_analysis_map() -> None:
        try:
            csv_path=normalize_path(analysis_path_var.get())
            if not os.path.isfile(csv_path): raise ValueError("Choose a valid CSV file.")
            metric=selected_analysis_metric()
            if not metric: raise ValueError("Choose a main parameter to analyse.")
            selected_ids=[field_id for field_id,var in analysis_timeline_vars.items() if bool(var.get())]
            rule_display=analysis_rule_var.get(); rule="higher" if rule_display.startswith("Higher") else "lower" if rule_display.startswith("Lower") else "bands"
            try:
                good=float(analysis_good_threshold_var.get()); bad=float(analysis_bad_threshold_var.get())
            except Exception:
                raise ValueError("Both analysis thresholds must be numbers.")
            defaults=_analysis_default_rule(metric,[])
            cfg={"rule":rule,"good_threshold":good,"bad_threshold":bad,"good_label":defaults["good_label"],"warn_label":defaults["warn_label"],"bad_label":defaults["bad_label"]}
            if rule=="bands": cfg.update({"good_label":"Low","warn_label":"Medium","bad_label":"High"})
            privacy_cfg=build_privacy_config_from_vars(analysis_privacy["mode"],analysis_privacy["blanket"],analysis_privacy["start"],analysis_privacy["end"],analysis_privacy["show"])
            initial_key=tile_display_to_key.get(analysis_initial_tile_var.get(),BUILTIN_PRESET_INITIAL_TILE_KEY)
            try:
                png_width=int(str(analysis_png_width_var.get()).strip())
                png_height=int(str(analysis_png_height_var.get()).strip())
            except Exception:
                raise ValueError("PNG width and height must be whole pixel values, such as 1920 and 1080.")
            if not (640 <= png_width <= 7680 and 360 <= png_height <= 4320):
                raise ValueError("PNG size must be between 640 × 360 and 7680 × 4320 pixels.")
            graph_export={"width":png_width,"height":png_height,"title":analysis_chart_title_var.get().strip(),"filename":analysis_png_filename_var.get().strip()}
            log(f"Building {metric['label']} analysis: {os.path.basename(csv_path)}")
            payload=build_flight_analysis_payload(csv_path,selected_ids,privacy_config=privacy_cfg,primary_metric=metric,analysis_config=cfg,status_callback=log)
            out_path=output_path_for_analysis_report(csv_path,metric)
            with open(out_path,"w",encoding="utf-8") as report_file:
                report_file.write(build_flight_analysis_html(payload,initial_tile_key=initial_key,graph_export=graph_export))
            summary=payload["summary"]
            saved_settings=load_parameter_settings()
            profiles=saved_settings.get("analysis_profiles",{}) if isinstance(saved_settings.get("analysis_profiles",{}),dict) else {}
            profiles[str(metric.get("id"))]={"rule":rule,"good_threshold":good,"bad_threshold":bad}
            saved_settings["analysis_profiles"]=profiles
            saved_settings["analysis_png_width"]=png_width
            saved_settings["analysis_png_height"]=png_height
            saved_settings["analysis_chart_title"]=analysis_chart_title_var.get().strip()
            saved_settings["analysis_png_filename"]=analysis_png_filename_var.get().strip()
            save_parameter_settings(saved_settings)
            log(f"Created analysis HTML: {out_path}")
            log(f"  Parameter: {metric['label']} | valid rows: {summary['metric_valid']}/{summary['samples']} | flagged episodes: {summary['flagged_runs']}")
        except Exception as exc:
            messagebox.showerror("Flight analysis failed",str(exc))

    analysis_buttons=ttk.Frame(analysis_inner); analysis_buttons.grid(row=10,column=0,columnspan=4,sticky="ew",padx=8,pady=8)
    ttk.Button(analysis_buttons,text="Generate interactive analysis HTML",command=generate_analysis_map).pack(side="right")
    analysis_metric_combo.bind("<<ComboboxSelected>>",refresh_analysis_parameter_defaults)
    analysis_rule_combo.bind("<<ComboboxSelected>>",update_analysis_rule_labels)
    _analysis_after_id: Optional[str]=None
    def _analysis_path_changed(*_args: Any) -> None:
        nonlocal _analysis_after_id
        if _analysis_after_id:
            try: root.after_cancel(_analysis_after_id)
            except Exception: pass
        _analysis_after_id=root.after(500,lambda:refresh_analysis_metrics(silent=True))
    analysis_path_var.trace_add("write",_analysis_path_changed)

    # ------------------------------------------------------------------
    # Tab 4: Dashware / added CSV columns and GPX
    # ------------------------------------------------------------------
    dashware_tab = ttk.Frame(notebook)
    notebook.add(dashware_tab, text="Dashware")
    dashware_tab.columnconfigure(0, weight=1); dashware_tab.rowconfigure(0, weight=1)
    dash_outer, dash_inner, dash_canvas = make_scrolled_frame(dashware_tab)
    dash_outer.grid(row=0, column=0, sticky="nsew"); dash_inner.columnconfigure(1, weight=1)

    ttk.Label(dash_inner, text="Add computed columns for Dashware and similar overlay software", font=("Segoe UI", 12, "bold")).grid(row=0,column=0,columnspan=4,sticky="w",padx=8,pady=(8,4))
    make_wrapped_label(dash_inner,"Every original header, row, timestamp, and data point is copied unchanged. Selected new columns are appended after TxBat. The source CSV is never edited. Outputs are saved beside it as '(Dashware).csv'.").grid(row=1,column=0,columnspan=4,sticky="ew",padx=8,pady=(0,6))

    dash_mode_var=tk.StringVar(value="Single CSV")
    dash_path_var=tk.StringVar(value=initial_csvs[0] if initial_csvs else (initial_folders[0] if initial_folders else ""))
    ttk.Label(dash_inner,text="Input mode").grid(row=2,column=0,sticky="w",padx=8,pady=4)
    make_combobox(dash_inner,textvariable=dash_mode_var,values=["Single CSV","Folder recursively"],state="readonly").grid(row=2,column=1,sticky="ew",padx=6,pady=4)
    ttk.Label(dash_inner,text="CSV file or folder").grid(row=3,column=0,sticky="w",padx=8,pady=4)
    dash_entry=ttk.Entry(dash_inner,textvariable=dash_path_var); dash_entry.grid(row=3,column=1,sticky="ew",padx=6,pady=4); add_dnd_to_entry(dash_entry,dash_path_var)
    def browse_dash_file() -> None:
        p=filedialog.askopenfilename(filetypes=[("CSV files","*.csv"),("All files","*.*")])
        if p: dash_mode_var.set("Single CSV"); dash_path_var.set(p)
    def browse_dash_folder() -> None:
        p=filedialog.askdirectory()
        if p: dash_mode_var.set("Folder recursively"); dash_path_var.set(p)
    dash_browse_frame=ttk.Frame(dash_inner); dash_browse_frame.grid(row=3,column=2,columnspan=2,sticky="e",padx=8,pady=4)
    ttk.Button(dash_browse_frame,text="Browse CSV",command=browse_dash_file).pack(side="left",padx=4)
    ttk.Button(dash_browse_frame,text="Browse folder",command=browse_dash_folder).pack(side="left",padx=4)

    saved_dash=load_parameter_settings()
    dash_throttle_var=tk.StringVar(value="CH3(us)")
    ttk.Label(dash_inner,text="Throttle channel").grid(row=4,column=0,sticky="w",padx=8,pady=4)
    dash_throttle_combo=make_combobox(dash_inner,textvariable=dash_throttle_var,values=["CH3(us)"],state="readonly")
    dash_throttle_combo.grid(row=4,column=1,columnspan=3,sticky="ew",padx=6,pady=4)

    terrain_box=ttk.LabelFrame(dash_inner,text="Terrain data source (shared across all modes)")
    terrain_box.grid(row=5,column=0,columnspan=4,sticky="ew",padx=8,pady=6); terrain_box.columnconfigure(1,weight=1)
    terrain_source_var=shared_terrain_source_var
    terrain_folder_var=shared_terrain_folder_var
    ttk.Label(terrain_box,text="Source").grid(row=0,column=0,sticky="w",padx=6,pady=4)
    make_combobox(terrain_box,textvariable=terrain_source_var,values=["Local terrain files","OpenTopoData online","Local first, then online fallback"],state="readonly").grid(row=0,column=1,columnspan=2,sticky="ew",padx=6,pady=4)
    terrain_folder_label=ttk.Label(terrain_box,text="Terrain folder"); terrain_folder_label.grid(row=1,column=0,sticky="w",padx=6,pady=4)
    terrain_folder_entry=ttk.Entry(terrain_box,textvariable=terrain_folder_var); terrain_folder_entry.grid(row=1,column=1,sticky="ew",padx=6,pady=4)
    def browse_terrain_folder() -> None:
        chosen=filedialog.askdirectory(title="Choose folder containing ArduPilot DAT or SRTM HGT terrain files",initialdir=terrain_folder_var.get() or None)
        if chosen: terrain_folder_var.set(chosen)
    terrain_folder_button=ttk.Button(terrain_box,text="Browse",command=browse_terrain_folder); terrain_folder_button.grid(row=1,column=2,sticky="e",padx=6,pady=4)
    terrain_status_var=tk.StringVar(value="")
    make_wrapped_label(terrain_box,"Local is the default and recursively scans N49W114.DAT-style ArduPilot terrain files and standard .HGT tiles. OpenTopoData is optional and may rate-limit large jobs.").grid(row=2,column=0,columnspan=3,sticky="ew",padx=6,pady=(0,4))
    def update_terrain_controls(*_args: Any) -> None:
        online_only=terrain_source_var.get().startswith("OpenTopoData")
        if online_only: terrain_folder_label.grid_remove(); terrain_folder_entry.grid_remove(); terrain_folder_button.grid_remove()
        else: terrain_folder_label.grid(); terrain_folder_entry.grid(); terrain_folder_button.grid()
    terrain_source_var.trace_add("write",update_terrain_controls); update_terrain_controls()

    dash_units_box=ttk.LabelFrame(dash_inner,text="Units, elapsed time, and precision")
    dash_units_box.grid(row=6,column=0,columnspan=4,sticky="ew",padx=8,pady=6); dash_units_box.columnconfigure(1,weight=1); dash_units_box.columnconfigure(3,weight=1)
    dash_unit_system_var=tk.StringVar(value=str(saved_dash.get("unit_system","Metric")))
    dash_elapsed_format_var=tk.StringVar(value=str(saved_dash.get("elapsed_format","Seconds")))
    ttk.Label(dash_units_box,text="Unit system").grid(row=0,column=0,sticky="w",padx=6,pady=4)
    dash_unit_system_combo=make_combobox(dash_units_box,textvariable=dash_unit_system_var,values=["Metric","Imperial","Custom"],state="readonly")
    dash_unit_system_combo.grid(row=0,column=1,sticky="ew",padx=6,pady=4)
    ttk.Label(dash_units_box,text="Elapsed-time format").grid(row=0,column=2,sticky="w",padx=6,pady=4)
    make_combobox(dash_units_box,textvariable=dash_elapsed_format_var,values=["Seconds","Decimal minutes","Clock H:MM:SS.mmm"],state="readonly").grid(row=0,column=3,sticky="ew",padx=6,pady=4)
    make_wrapped_label(dash_units_box,"Seconds is the default and follows the original Time column down to its logged milliseconds. Decimal minutes means 1.1 at 1 minute 6 seconds. Clock format writes H:MM:SS.mmm.").grid(row=1,column=0,columnspan=4,sticky="ew",padx=6,pady=(0,4))

    custom_units=ttk.Frame(dash_units_box); custom_units.grid(row=2,column=0,columnspan=4,sticky="ew",padx=4,pady=2)
    for col in (1,3): custom_units.columnconfigure(col,weight=1)
    dash_distance_unit_var=tk.StringVar(value=str(saved_dash.get("distance_unit","m")))
    dash_long_distance_unit_var=tk.StringVar(value=str(saved_dash.get("long_distance_unit","km")))
    dash_speed_unit_var=tk.StringVar(value=str(saved_dash.get("speed_unit","km/h")))
    dash_altitude_unit_var=tk.StringVar(value=str(saved_dash.get("altitude_unit","m")))
    dash_vertical_unit_var=tk.StringVar(value=str(saved_dash.get("vertical_speed_unit","m/s")))
    dash_accel_unit_var=tk.StringVar(value=str(saved_dash.get("acceleration_unit","m/s²")))
    dash_angular_rate_var=tk.StringVar(value=str(saved_dash.get("angular_rate_unit","deg/s")))
    dash_eff_distance_var=tk.StringVar(value=str(saved_dash.get("efficiency_distance_unit","km")))
    custom_specs=[
        ("Distance from home",dash_distance_unit_var,["m","km","ft","mi"]),("Cumulative distance",dash_long_distance_unit_var,["km","mi","m","ft"]),
        ("Speed",dash_speed_unit_var,["km/h","mph","m/s","kn"]),("Altitude",dash_altitude_unit_var,["m","ft","km","mi"]),
        ("Vertical speed",dash_vertical_unit_var,["m/s","ft/s","ft/min"]),("Acceleration",dash_accel_unit_var,["m/s²","ft/s²"]),
        ("Efficiency distance",dash_eff_distance_var,["km","mi"]),
    ]
    for i,(label,var,values) in enumerate(custom_specs):
        row=i//2; col=(i%2)*2; ttk.Label(custom_units,text=label).grid(row=row,column=col,sticky="w",padx=4,pady=2)
        make_combobox(custom_units,textvariable=var,values=values,state="readonly").grid(row=row,column=col+1,sticky="ew",padx=4,pady=2)

    precision_frame=ttk.Frame(dash_units_box); precision_frame.grid(row=3,column=0,columnspan=4,sticky="ew",padx=4,pady=2)
    precision_vars={
        "short_decimals":tk.StringVar(value=str(saved_dash.get("short_decimals",0))),"long_decimals":tk.StringVar(value=str(saved_dash.get("long_decimals",2))),
        "altitude_decimals":tk.StringVar(value=str(saved_dash.get("altitude_decimals",1))),"speed_decimals":tk.StringVar(value=str(saved_dash.get("speed_decimals",1))),
        "general_decimals":tk.StringVar(value=str(saved_dash.get("general_decimals",2))),
    }
    precision_labels=[("Metres/feet", "short_decimals"),("Kilometres/miles", "long_decimals"),("Altitude", "altitude_decimals"),("Speed", "speed_decimals"),("Other", "general_decimals")]
    for i,(label,key) in enumerate(precision_labels):
        ttk.Label(precision_frame,text=f"{label} decimals").grid(row=0,column=i*2,sticky="w",padx=(4,2),pady=2)
        make_combobox(precision_frame,textvariable=precision_vars[key],values=[str(n) for n in range(0,7)],state="readonly",width=4).grid(row=0,column=i*2+1,sticky="w",padx=(0,8),pady=2)
    angular_frame=ttk.Frame(dash_units_box); angular_frame.grid(row=4,column=0,columnspan=4,sticky="ew",padx=4,pady=2)
    ttk.Label(angular_frame,text="Angular-rate unit").pack(side="left",padx=(4,6))
    make_combobox(angular_frame,textvariable=dash_angular_rate_var,values=["deg/s","rad/s"],state="readonly",width=10).pack(side="left")
    ttk.Label(angular_frame,text="Used for ground-track turn rate and roll/pitch/yaw rates.").pack(side="left",padx=8)
    dash_joke_var=tk.BooleanVar(value=bool(saved_dash.get("joke_altitude_cap",False)))
    ttk.Checkbutton(dash_units_box,text="Joke mode: cap only generated altitude columns above 400 ft (122 m). Original Alt data remains untouched.",variable=dash_joke_var).grid(row=5,column=0,columnspan=4,sticky="w",padx=6,pady=4)

    def update_custom_unit_visibility(*_args: Any) -> None:
        if dash_unit_system_var.get()=="Custom": custom_units.grid()
        else: custom_units.grid_remove()
    dash_unit_system_combo.bind("<<ComboboxSelected>>",update_custom_unit_visibility); update_custom_unit_visibility()

    def collect_dash_settings() -> Dict[str,Any]:
        result={"unit_system":dash_unit_system_var.get(),"elapsed_format":dash_elapsed_format_var.get(),"distance_unit":dash_distance_unit_var.get(),
            "long_distance_unit":dash_long_distance_unit_var.get(),"speed_unit":dash_speed_unit_var.get(),"altitude_unit":dash_altitude_unit_var.get(),
            "vertical_speed_unit":dash_vertical_unit_var.get(),"acceleration_unit":dash_accel_unit_var.get(),"angular_rate_unit":dash_angular_rate_var.get(),
            "efficiency_distance_unit":dash_eff_distance_var.get(),"joke_altitude_cap":bool(dash_joke_var.get()),"clamp_negative_agl":True,"terrain_source":terrain_source_var.get(),"terrain_folder":terrain_folder_var.get().strip()}
        for key,var in precision_vars.items(): result[key]=int(var.get())
        return result
    def save_dash_settings_gui() -> None:
        try: save_parameter_settings(collect_dash_settings()); log("Saved Dashware, terrain-source, unit, and precision settings to the preset/settings JSON.")
        except Exception as exc: messagebox.showerror("Settings save failed",str(exc))
    ttk.Button(dash_units_box,text="Save these unit/precision choices",command=save_dash_settings_gui).grid(row=6,column=3,sticky="e",padx=6,pady=4)

    dash_fields_box=ttk.LabelFrame(dash_inner,text="Columns to append after the original TxBat column")
    dash_fields_box.grid(row=7,column=0,columnspan=4,sticky="ew",padx=8,pady=6); dash_fields_box.columnconfigure(0,weight=1)
    make_wrapped_label(dash_fields_box,"Alt MSL uses terrain elevation at the first valid zero-relative-altitude GPS sample as takeoff elevation. Alt AGL subtracts terrain at each current GPS point. Terrain values use the selected local or online terrain source. Local terrain mode queries every distinct logged GPS coordinate. Vertical speed is most accurate when Alt comes from a barometer rather than GPS.").grid(row=0,column=0,sticky="ew",padx=6,pady=4)
    dash_fields_scroll,dash_fields_inner,dash_fields_canvas=make_scrolled_frame(dash_fields_box,height=410)
    dash_fields_scroll.grid(row=1,column=0,sticky="ew",padx=6,pady=4); dash_fields_inner.columnconfigure(0,weight=1)
    dash_field_vars: Dict[str,Any]={}
    saved_field_ids_raw=saved_dash.get("dashware_selected_fields",[])
    saved_field_ids={str(x) for x in saved_field_ids_raw} if isinstance(saved_field_ids_raw,list) and saved_field_ids_raw else set(DASHWARE_DEFAULT_FIELD_IDS)
    for i,field in enumerate(DASHWARE_FIELDS):
        var=tk.BooleanVar(value=field["id"] in saved_field_ids); dash_field_vars[field["id"]]=var
        cb=ttk.Checkbutton(dash_fields_inner,text=field["label"],variable=var)
        if field["id"]=="elapsed": cb.configure(state="disabled"); var.set(True)
        cb.grid(row=i,column=0,sticky="w",padx=8,pady=1); _mark_mousewheel(cb,dash_fields_canvas)
    def set_dash_fields(mode: str) -> None:
        for field_id,var in dash_field_vars.items():
            if field_id=="elapsed": var.set(True)
            elif mode=="all": var.set(True)
            elif mode=="none": var.set(False)
            else: var.set(field_id in DASHWARE_DEFAULT_FIELD_IDS)
    dash_select_buttons=ttk.Frame(dash_fields_box); dash_select_buttons.grid(row=2,column=0,sticky="w",padx=6,pady=(0,4))
    ttk.Button(dash_select_buttons,text="Recommended",command=lambda:set_dash_fields("default")).pack(side="left",padx=(0,6))
    ttk.Button(dash_select_buttons,text="Select all",command=lambda:set_dash_fields("all")).pack(side="left",padx=6)
    ttk.Button(dash_select_buttons,text="Clear all",command=lambda:set_dash_fields("none")).pack(side="left",padx=6)
    def save_dash_fields_gui() -> None:
        try:
            settings=collect_dash_settings()
            settings["dashware_selected_fields"]=[fid for fid,var in dash_field_vars.items() if bool(var.get())]
            save_parameter_settings(settings)
            log("Saved Dashware column selections to the preset/settings JSON.")
        except Exception as exc:
            messagebox.showerror("Column-selection save failed",str(exc))
    ttk.Button(dash_select_buttons,text="Save selected columns",command=save_dash_fields_gui).pack(side="left",padx=6)
    dash_gpx_var=tk.BooleanVar(value=True)
    ttk.Checkbutton(dash_inner,text="Also create a GPX track beside each CSV",variable=dash_gpx_var).grid(row=8,column=0,columnspan=4,sticky="w",padx=8,pady=4)

    def dash_input_csvs() -> List[str]:
        path=normalize_path(dash_path_var.get())
        if dash_mode_var.get().startswith("Folder"):
            if not os.path.isdir(path): raise ValueError("Choose a valid folder.")
            return [p for p in find_csv_files(path) if "(dashware" not in os.path.basename(p).lower()]
        if not os.path.isfile(path) or not path.lower().endswith(".csv"): raise ValueError("Choose a valid CSV file.")
        return [path]
    def refresh_dash_throttle(silent: bool=True) -> None:
        try:
            csvs=dash_input_csvs(); header=get_csv_header(csvs[0]) if csvs else []; channels=available_channel_columns_from_header(header) or ["CH3(us)"]
            dash_throttle_combo.configure(values=channels)
            if dash_throttle_var.get() not in channels: dash_throttle_var.set("CH3(us)" if "CH3(us)" in channels else channels[0])
        except Exception as exc:
            if not silent: messagebox.showerror("Throttle-channel load failed",str(exc))
    def generate_dashware_outputs() -> None:
        try:
            csvs=dash_input_csvs(); selected=[fid for fid,var in dash_field_vars.items() if bool(var.get())]
            settings=collect_dash_settings(); settings["dashware_selected_fields"]=list(selected); save_parameter_settings(settings)
            log(f"Dashware processing: {len(csvs)} CSV file(s)")
            made_csv=made_gpx=skipped=0
            for i,csv_path in enumerate(csvs,start=1):
                try:
                    log(f"[{i}/{len(csvs)}] Processing {os.path.basename(csv_path)}")
                    result=enrich_csv_for_dashware(csv_path,selected,create_gpx=bool(dash_gpx_var.get()),throttle_col_name=dash_throttle_var.get(),status_callback=log,parameter_settings=settings)
                    made_csv+=1; made_gpx+=1 if result.get("gpx") else 0
                    log(f"  Created CSV: {result['csv']}")
                    if result.get("gpx"): log(f"  Created GPX: {result['gpx']}")
                    if result.get("terrain_source")!="Not requested": log(f"  Terrain source: {result['terrain_source']}")
                    if result.get("takeoff_source")!="Not requested":
                        text="unavailable" if result.get("takeoff_msl") is None else f"{float(result['takeoff_msl']):.1f} m"
                        log(f"  Terrain takeoff reference: {text} ({result['takeoff_source']})")
                except Exception as file_exc:
                    skipped+=1; log(f"  Skipped: {file_exc}")
            log(f"Dashware processing complete. Created {made_csv} CSV(s) and {made_gpx} GPX file(s); skipped {skipped} file(s).")
        except Exception as exc: messagebox.showerror("Dashware processing failed",str(exc))
    dash_buttons=ttk.Frame(dash_inner); dash_buttons.grid(row=9,column=0,columnspan=4,sticky="ew",padx=8,pady=8)
    ttk.Button(dash_buttons,text="Generate Dashware CSV / GPX",command=generate_dashware_outputs).pack(side="right")
    _dash_after_id: Optional[str]=None
    def _dash_path_changed(*_args: Any) -> None:
        nonlocal _dash_after_id
        if _dash_after_id:
            try: root.after_cancel(_dash_after_id)
            except Exception: pass
        _dash_after_id=root.after(500,lambda:refresh_dash_throttle(silent=True))
    dash_path_var.trace_add("write",_dash_path_changed); dash_mode_var.trace_add("write",_dash_path_changed)

    # ------------------------------------------------------------------
    # Tab 4: all flights summary
    # ------------------------------------------------------------------
    summary_tab = ttk.Frame(notebook)
    notebook.add(summary_tab, text="All flights summary")
    summary_tab.columnconfigure(0, weight=1)
    summary_tab.rowconfigure(2, weight=1)
    summary_tab.rowconfigure(5, weight=1)
    summary_tab.rowconfigure(7, weight=2)

    ttk.Label(summary_tab, text="All flights summary", font=("Segoe UI", 12, "bold")).grid(row=0, column=0, sticky="w", pady=(8, 4), padx=8)
    make_wrapped_label(summary_tab, "Add folders/files containing ArduPilot + Betaflight EdgeTX CSV logs. Folder scanning is recursive. Date prefers the CSV Date column; filename date is fallback only.").grid(row=1, column=0, sticky="ew", padx=8)
    skip_non_flight_var = tk.BooleanVar(value=True)
    summary_include_four_sat_var = tk.BooleanVar(value=False)

    paths_frame, paths_text = make_scrolled_text(summary_tab, height=5, wrap="none")
    paths_frame.grid(row=2, column=0, sticky="nsew", pady=6, padx=8)
    add_dnd_to_text(paths_text)
    for p in initial_paths:
        paths_text.insert("end", p + "\n")

    paths_buttons = ttk.Frame(summary_tab)
    paths_buttons.grid(row=3, column=0, sticky="ew", padx=8)
    def add_summary_folder() -> None:
        p = filedialog.askdirectory(title="Choose folder of CSV logs")
        if p:
            paths_text.insert("end", p + "\n")
    def add_summary_files() -> None:
        ps = filedialog.askopenfilenames(title="Choose CSV logs", filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
        for p in ps:
            paths_text.insert("end", p + "\n")
    ttk.Button(paths_buttons, text="Add folder", command=add_summary_folder).pack(side="left", padx=(0, 6))
    ttk.Button(paths_buttons, text="Add CSV files", command=add_summary_files).pack(side="left", padx=(0, 6))
    ttk.Button(paths_buttons, text="Clear paths", command=lambda: paths_text.delete("1.0", "end")).pack(side="left")
    ttk.Button(paths_buttons, text="Scan flights / detect aircraft names", command=lambda: scan_summary_gui()).pack(side="left", padx=(6, 0))
    ttk.Checkbutton(paths_buttons, text="Skip CSV files that do not look like flight logs (recommended)", variable=skip_non_flight_var).pack(side="left", padx=(14, 0))
    ttk.Checkbutton(paths_buttons, text="Include 4-satellite GPS rows (5+ recommended)", variable=summary_include_four_sat_var).pack(side="left", padx=(14, 0))

    make_wrapped_label(summary_tab, "Aircraft grouping mapping — edit right side as needed. Format: detected name = aircraft group").grid(row=4, column=0, sticky="ew", pady=(12, 0), padx=8)
    mapping_frame, mapping_text = make_scrolled_text(summary_tab, height=8, wrap="none")
    mapping_frame.grid(row=5, column=0, sticky="nsew", pady=6, padx=8)

    filter_frame = ttk.Frame(summary_tab)
    filter_frame.grid(row=6, column=0, sticky="ew", pady=(0, 6), padx=8)
    month_var = tk.StringVar()
    ttk.Label(filter_frame, text="Optional filter:").pack(side="left")
    ttk.Entry(filter_frame, textvariable=month_var, width=16).pack(side="left", padx=6)
    ttk.Label(filter_frame, text="blank = all, or type 2025 / 2025-07").pack(side="left")

    summary_frame, summary_output = make_scrolled_text(summary_tab, height=10, wrap="word")
    summary_frame.grid(row=7, column=0, sticky="nsew", pady=6, padx=8)

    gui_summary_records: List[Dict[str, Any]] = []
    def _summary_threshold_changed(*_args: Any) -> None:
        nonlocal gui_summary_records
        if gui_summary_records:
            gui_summary_records = []
            log("All-flights GPS threshold changed. Scan flights again before generating the summary.")
    summary_include_four_sat_var.trace_add("write", _summary_threshold_changed)

    def _get_summary_paths() -> List[str]:
        return [normalize_path(line.strip()) for line in paths_text.get("1.0", "end").splitlines() if line.strip()]

    def scan_summary_gui() -> None:
        nonlocal gui_summary_records
        paths = _get_summary_paths()
        csvs = csv_files_from_paths(paths)
        if not csvs:
            messagebox.showwarning("No CSV files", "No CSV files found in the listed path(s).")
            return
        old_stdout = sys.stdout
        try:
            sys.stdout = _TextRedirector(log_text)
            summary_min_sats = RELAXED_MIN_SATS if bool(summary_include_four_sat_var.get()) else MIN_SATS
            gui_summary_records = scan_flights_for_summary(csvs, skip_non_flight_logs=bool(skip_non_flight_var.get()), min_sats=summary_min_sats)
        finally:
            sys.stdout = old_stdout
        raw_names = sorted({r["aircraft_raw"] for r in gui_summary_records})
        suggested = _suggest_aircraft_groups(raw_names)
        remembered = load_aircraft_group_mapping()
        mapping_text.delete("1.0", "end")
        for name in raw_names:
            mapping_text.insert("end", f"{name} = {remembered.get(name, suggested.get(name, name))}\n")
        threshold = RELAXED_MIN_SATS if bool(summary_include_four_sat_var.get()) else MIN_SATS
        log(f"Scanned {len(gui_summary_records)} usable flight record(s) using a {threshold}+ satellite GPS-good threshold. Review aircraft grouping, delete unwanted lines, then generate report.")

    def _read_mapping_from_gui() -> Dict[str, str]:
        mapping: Dict[str, str] = {}
        for line in mapping_text.get("1.0", "end").splitlines():
            if "=" in line:
                left, right = line.split("=", 1)
                left = left.strip()
                right = right.strip()
                if left and right:
                    mapping[left] = right
        return mapping

    def generate_summary_gui() -> None:
        nonlocal gui_summary_records
        if not gui_summary_records:
            scan_summary_gui()
        if not gui_summary_records:
            return
        mapping = _read_mapping_from_gui()
        if not mapping:
            messagebox.showwarning("No aircraft groups", "The aircraft grouping box is empty. Scan flights again or leave at least one group line.")
            return
        save_aircraft_group_mapping(mapping, merge=True)
        records = apply_aircraft_mapping(gui_summary_records, mapping, only_mapped=True)
        if not records:
            messagebox.showwarning("No flights selected", "No scanned flights matched the remaining aircraft grouping lines.")
            return
        report = build_all_flights_summary_report(records, month_filter=month_var.get())
        summary_output.delete("1.0", "end")
        summary_output.insert("end", report)
        log("Summary report generated. Aircraft grouping preferences were saved.")

    def save_summary_gui() -> None:
        report = summary_output.get("1.0", "end").strip()
        if not report:
            messagebox.showwarning("No report", "Generate a summary first.")
            return
        out = filedialog.asksaveasfilename(title="Save summary report", defaultextension=".txt", filetypes=[("Text files", "*.txt"), ("All files", "*.*")])
        if not out:
            return
        with open(out, "w", encoding="utf-8") as f:
            f.write(report + "\n")
        log(f"Saved summary: {out}")

    add_shared_terrain_controls(summary_tab, row=9, columnspan=1)
    summary_buttons = ttk.Frame(summary_tab)
    summary_buttons.grid(row=8, column=0, sticky="ew", pady=(4, 8), padx=8)
    ttk.Button(summary_buttons, text="Save report as TXT", command=save_summary_gui).pack(side="right", padx=(6, 0))
    ttk.Button(summary_buttons, text="Generate summary", command=generate_summary_gui).pack(side="right", padx=(6, 0))

    # ------------------------------------------------------------------
    # Tab 5: single 3D KMZ map
    # ------------------------------------------------------------------
    def _kmz_elevation_options_from_vars(mode_var: Any, manual_var: Any, include_four_var: Any = None, confirm_var: Any = None, offset: float = 0.0) -> KMZAltitudeOptions:
        mode_display = str(mode_var.get()).lower()
        min_sats = RELAXED_MIN_SATS if include_four_var is not None and bool(include_four_var.get()) else MIN_SATS
        if mode_display.startswith("manual"):
            try:
                manual = float(str(manual_var.get()).strip())
            except Exception:
                raise ValueError("Type a manual takeoff elevation in metres, such as 1300.")
            return KMZAltitudeOptions(mode="manual", manual_takeoff_msl=manual, visual_offset_m=float(offset), confirm_online=False, min_sats=min_sats)
        if mode_display.startswith("csv"):
            return KMZAltitudeOptions(mode="csv_only", manual_takeoff_msl=None, visual_offset_m=float(offset), confirm_online=False, min_sats=min_sats)
        return KMZAltitudeOptions(mode="auto", manual_takeoff_msl=None, visual_offset_m=float(offset), confirm_online=False, min_sats=min_sats)

    def _kmz_manual_fallback_dialog(csv_path: str, sample: KMZFirstSample, reason: str) -> Optional[float]:
        prompt = f"Takeoff elevation needed for:\n{os.path.basename(csv_path)}\n\nFirst good GPS: {sample.lat:.7f}, {sample.lon:.7f}\nReason: {reason}\n\nType the takeoff elevation above sea level in metres."
        while True:
            value = simpledialog.askstring("Takeoff elevation needed", prompt, parent=root)
            if value is None:
                return None
            try:
                return float(value.strip())
            except Exception:
                messagebox.showerror("Invalid elevation", "Please type a number like 1300 or 936.5.")

    def _kmz_confirm_elevation_dialog(csv_path: str, sample: KMZFirstSample, found_elev: float, source: str) -> Optional[float]:
        msg = (
            f"File: {os.path.basename(csv_path)}\n"
            f"First good GPS: {sample.lat:.7f}, {sample.lon:.7f}\n"
            f"Found takeoff elevation: {found_elev:.1f} m ({source})\n\n"
            "Use this elevation? Choose Yes to use it, No to type a different value, or Cancel to stop."
        )
        answer = messagebox.askyesnocancel("Confirm takeoff elevation", msg, parent=root)
        if answer is None:
            return None
        if answer is True:
            return float(found_elev)
        return _kmz_manual_fallback_dialog(csv_path, sample, "Manual override requested")

    def _kmz_files_from_text(text_widget: Any) -> List[str]:
        raw_paths = [normalize_path(line.strip()) for line in text_widget.get("1.0", "end").splitlines() if line.strip()]
        csvs: List[str] = []
        for p in raw_paths:
            if os.path.isdir(p):
                csvs.extend(find_csv_files(p))
            elif os.path.isfile(p) and p.lower().endswith(".csv"):
                csvs.append(p)
        seen = set()
        out: List[str] = []
        for p in csvs:
            if p not in seen:
                out.append(p)
                seen.add(p)
        return out

    kmz_session_outputs: List[str] = []

    def remember_kmz_output(path: Optional[str]) -> None:
        if path:
            ap = os.path.abspath(path)
            if ap not in kmz_session_outputs:
                kmz_session_outputs.append(ap)

    def recycle_session_kmz_outputs() -> None:
        operation_start("KMZ session Recycle Bin cleanup")
        if not kmz_session_outputs:
            messagebox.showinfo("No KMZ outputs", "No KMZ files have been created in this app session yet.")
            return
        if not messagebox.askyesno(
            "Move KMZ outputs to Recycle Bin",
            f"Move {len(kmz_session_outputs)} KMZ file path(s) created in this app session to the Recycle Bin?\n\n"
            "Only those exact files will be touched. Missing files will be skipped.",
            parent=root,
        ):
            return
        moved, skipped = move_paths_to_recycle_bin(kmz_session_outputs, status_callback=log)
        remaining = [p for p in kmz_session_outputs if os.path.isfile(p)]
        kmz_session_outputs[:] = remaining
        log(f"Recycle Bin cleanup finished. Moved: {moved}, skipped/missing: {skipped}, still existing: {len(remaining)}")

    single_kmz_tab = ttk.Frame(notebook)
    notebook.add(single_kmz_tab, text="Single 3D map")
    single_kmz_tab.columnconfigure(1, weight=1)
    single_kmz_tab.rowconfigure(7, weight=1)
    ttk.Label(single_kmz_tab, text="Single 3D KMZ map for Google Earth", font=("Segoe UI", 12, "bold")).grid(row=0, column=0, columnspan=3, sticky="w", padx=8, pady=(8, 4))
    make_wrapped_label(single_kmz_tab, "Create a 3D KMZ from one Betaflight or ArduPilot-style EdgeTX CSV. The path breaks when GPS is missing, satellites are below the selected threshold, or altitude is missing.").grid(row=1, column=0, columnspan=3, sticky="ew", padx=8, pady=(0, 4))
    single_kmz_path_var = tk.StringVar(value=initial_csvs[0] if initial_csvs else "")
    single_kmz_four_sat_var = tk.BooleanVar(value=False)
    ttk.Label(single_kmz_tab, text="CSV file").grid(row=2, column=0, sticky="w", padx=8, pady=4)
    single_kmz_entry = ttk.Entry(single_kmz_tab, textvariable=single_kmz_path_var)
    single_kmz_entry.grid(row=2, column=1, sticky="ew", padx=6, pady=4)
    add_dnd_to_entry(single_kmz_entry, single_kmz_path_var)
    def browse_single_kmz_csv() -> None:
        p = filedialog.askopenfilename(title="Choose CSV file", filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
        if p:
            single_kmz_path_var.set(p)
    ttk.Button(single_kmz_tab, text="Browse", command=browse_single_kmz_csv).grid(row=2, column=2, sticky="ew", padx=8, pady=4)
    ttk.Checkbutton(single_kmz_tab, text="Include track rows with exactly 4 satellites (5+ recommended; GPS altitude may freeze or jump)", variable=single_kmz_four_sat_var).grid(row=3, column=0, columnspan=3, sticky="w", padx=8, pady=4)

    kmz_ref_box = ttk.LabelFrame(single_kmz_tab, text="Takeoff elevation source")
    kmz_ref_box.grid(row=4, column=0, columnspan=3, sticky="ew", padx=8, pady=6)
    kmz_ref_box.columnconfigure(1, weight=1)
    single_kmz_mode_var = tk.StringVar(value="Automatic selected terrain source for Betaflight; ArduPilot ASL uses CSV")
    single_kmz_manual_var = tk.StringVar(value="")
    single_kmz_confirm_var = tk.BooleanVar(value=False)
    ttk.Label(kmz_ref_box, text="Mode").grid(row=0, column=0, sticky="w", padx=4, pady=2)
    single_kmz_mode_combo = make_combobox(kmz_ref_box, textvariable=single_kmz_mode_var, values=["Automatic selected terrain source for Betaflight; ArduPilot ASL uses CSV", "Manual takeoff elevation", "CSV altitude logic only"])
    single_kmz_mode_combo.grid(row=0, column=1, sticky="ew", padx=4, pady=2)
    single_kmz_manual_label = ttk.Label(kmz_ref_box, text="Manual elevation (m)")
    single_kmz_manual_label.grid(row=2, column=0, sticky="w", padx=4, pady=2)
    single_kmz_manual_entry = ttk.Entry(kmz_ref_box, textvariable=single_kmz_manual_var)
    single_kmz_manual_entry.grid(row=2, column=1, sticky="ew", padx=4, pady=2)
    def update_single_kmz_ref_visibility(*_args: Any) -> None:
        mode = single_kmz_mode_var.get().lower()
        if mode.startswith("manual"):
            single_kmz_manual_label.grid()
            single_kmz_manual_entry.grid()
        elif mode.startswith("csv"):
            single_kmz_manual_label.grid_remove()
            single_kmz_manual_entry.grid_remove()
        else:
            single_kmz_manual_label.grid_remove()
            single_kmz_manual_entry.grid_remove()
    single_kmz_mode_var.trace_add("write", update_single_kmz_ref_visibility)
    update_single_kmz_ref_visibility()

    add_shared_terrain_controls(single_kmz_tab, row=5, columnspan=3)
    single_kmz_comp_box = ttk.LabelFrame(single_kmz_tab, text="After checking the KMZ in Google Earth")
    single_kmz_comp_box.grid(row=6, column=0, columnspan=3, sticky="ew", padx=8, pady=6)
    single_kmz_comp_box.columnconfigure(1, weight=1)
    make_wrapped_label(single_kmz_comp_box, "If the track is slightly hidden by terrain or floating too high, type an altitude compensation in metres and generate another KMZ. Positive raises the track; negative lowers it.").grid(row=0, column=0, columnspan=3, sticky="ew", padx=4, pady=2)
    single_kmz_comp_var = tk.StringVar(value="")
    ttk.Label(single_kmz_comp_box, text="Compensation (m)").grid(row=1, column=0, sticky="w", padx=4, pady=2)
    ttk.Entry(single_kmz_comp_box, textvariable=single_kmz_comp_var, width=14).grid(row=1, column=1, sticky="w", padx=4, pady=2)

    single_kmz_last: Dict[str, Any] = {"csv": None, "outputs": []}
    def _single_kmz_base_options(offset: float = 0.0) -> KMZAltitudeOptions:
        save_parameter_settings(collect_dash_settings())
        return _kmz_elevation_options_from_vars(single_kmz_mode_var, single_kmz_manual_var, single_kmz_four_sat_var, single_kmz_confirm_var, offset=offset)
    def generate_single_kmz_initial() -> None:
        try:
            csv_path = normalize_path(single_kmz_path_var.get())
            if not os.path.isfile(csv_path):
                raise ValueError("Choose a valid CSV file. Pasted Copy as path values with quotes are okay.")
            opts = _single_kmz_base_options(offset=0.0)
            if opts.mode == "auto" and not terrain_preflight("Single 3D map export", True): return
            operation_start(f"Single 3D KMZ export: {csv_path}")
            out = kmz_write_kmz(csv_path, opts, manual_fallback_callback=_kmz_manual_fallback_dialog, status_callback=log)
            single_kmz_last["csv"] = csv_path
            if out:
                remember_kmz_output(out)
                single_kmz_last["outputs"] = [out]
                log("Open the KMZ in Google Earth. If the altitude needs a small visual adjustment, use the compensation box on this tab.")
        except Exception as exc:
            messagebox.showerror("3D KMZ export failed", str(exc))
    def generate_single_kmz_compensated() -> None:
        try:
            csv_path = single_kmz_last.get("csv") or normalize_path(single_kmz_path_var.get())
            if not os.path.isfile(csv_path):
                raise ValueError("Generate an initial KMZ first, or choose a valid CSV file.")
            try:
                offset = float(str(single_kmz_comp_var.get()).strip())
            except Exception:
                raise ValueError("Type an altitude compensation in metres, such as 5, 12.5, or -3.")
            opts = _single_kmz_base_options(offset=offset)
            suffix = kmz_safe_offset_suffix(offset)
            out = kmz_write_kmz(csv_path, opts, output_suffix=suffix, manual_fallback_callback=_kmz_manual_fallback_dialog, status_callback=log)
            if out:
                remember_kmz_output(out)
                single_kmz_last.setdefault("outputs", []).append(out)
        except Exception as exc:
            messagebox.showerror("Compensated KMZ export failed", str(exc))
    single_kmz_buttons = ttk.Frame(single_kmz_tab)
    single_kmz_buttons.grid(row=8, column=0, columnspan=3, sticky="ew", padx=8, pady=6)
    ttk.Button(single_kmz_buttons, text="Move session KMZs to Recycle Bin", command=recycle_session_kmz_outputs).pack(side="left", padx=(0, 8))
    ttk.Button(single_kmz_buttons, text="Generate compensated KMZ", command=generate_single_kmz_compensated).pack(side="right", padx=(8, 0))
    ttk.Button(single_kmz_buttons, text="Generate initial KMZ", command=generate_single_kmz_initial).pack(side="right", padx=(8, 0))

    # ------------------------------------------------------------------
    # Tab 6: multiple 3D KMZ maps
    # ------------------------------------------------------------------
    multi_kmz_tab = ttk.Frame(notebook)
    notebook.add(multi_kmz_tab, text="Multiple 3D maps")
    multi_kmz_tab.columnconfigure(0, weight=1)
    multi_kmz_tab.rowconfigure(2, weight=1)
    ttk.Label(multi_kmz_tab, text="Multiple 3D KMZ maps for Google Earth", font=("Segoe UI", 12, "bold")).grid(row=0, column=0, sticky="w", padx=8, pady=(8, 4))
    make_wrapped_label(multi_kmz_tab, "Add folders and/or CSV files. Folder scanning is recursive. KMZ files are written beside each CSV.").grid(row=1, column=0, sticky="ew", padx=8, pady=(0, 4))
    multi_kmz_paths_frame, multi_kmz_paths_text = make_scrolled_text(multi_kmz_tab, height=6, wrap="none")
    multi_kmz_paths_frame.grid(row=2, column=0, sticky="nsew", padx=8, pady=6)
    add_dnd_to_text(multi_kmz_paths_text)
    for p in initial_paths:
        multi_kmz_paths_text.insert("end", p + "\n")
    multi_kmz_path_buttons = ttk.Frame(multi_kmz_tab)
    multi_kmz_path_buttons.grid(row=3, column=0, sticky="ew", padx=8)
    def add_multi_kmz_folder() -> None:
        p = filedialog.askdirectory(title="Choose folder of CSV logs")
        if p:
            multi_kmz_paths_text.insert("end", p + "\n")
    def add_multi_kmz_files() -> None:
        ps = filedialog.askopenfilenames(title="Choose CSV logs", filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
        for p in ps:
            multi_kmz_paths_text.insert("end", p + "\n")
    ttk.Button(multi_kmz_path_buttons, text="Add folder", command=add_multi_kmz_folder).pack(side="left", padx=(0, 6))
    ttk.Button(multi_kmz_path_buttons, text="Add CSV files", command=add_multi_kmz_files).pack(side="left", padx=(0, 6))
    ttk.Button(multi_kmz_path_buttons, text="Clear paths", command=lambda: multi_kmz_paths_text.delete("1.0", "end")).pack(side="left")

    add_shared_terrain_controls(multi_kmz_tab, row=3, columnspan=1)
    multi_kmz_ref_box = ttk.LabelFrame(multi_kmz_tab, text="Takeoff elevation source")
    multi_kmz_ref_box.grid(row=4, column=0, sticky="ew", padx=8, pady=6)
    multi_kmz_ref_box.columnconfigure(1, weight=1)
    multi_kmz_mode_var = tk.StringVar(value="Automatic selected terrain source for Betaflight; ArduPilot ASL uses CSV")
    multi_kmz_manual_var = tk.StringVar(value="")
    multi_kmz_four_sat_var = tk.BooleanVar(value=False)
    multi_kmz_confirm_var = tk.BooleanVar(value=False)
    ttk.Label(multi_kmz_ref_box, text="Mode").grid(row=0, column=0, sticky="w", padx=4, pady=2)
    make_combobox(multi_kmz_ref_box, textvariable=multi_kmz_mode_var, values=["Automatic selected terrain source for Betaflight; ArduPilot ASL uses CSV", "Manual takeoff elevation", "CSV altitude logic only"]).grid(row=0, column=1, sticky="ew", padx=4, pady=2)
    multi_kmz_manual_label = ttk.Label(multi_kmz_ref_box, text="Manual elevation (m)")
    multi_kmz_manual_label.grid(row=2, column=0, sticky="w", padx=4, pady=2)
    multi_kmz_manual_entry = ttk.Entry(multi_kmz_ref_box, textvariable=multi_kmz_manual_var)
    multi_kmz_manual_entry.grid(row=2, column=1, sticky="ew", padx=4, pady=2)
    def update_multi_kmz_ref_visibility(*_args: Any) -> None:
        mode = multi_kmz_mode_var.get().lower()
        if mode.startswith("manual"):
            multi_kmz_manual_label.grid()
            multi_kmz_manual_entry.grid()
        elif mode.startswith("csv"):
            multi_kmz_manual_label.grid_remove()
            multi_kmz_manual_entry.grid_remove()
        else:
            multi_kmz_manual_label.grid_remove()
            multi_kmz_manual_entry.grid_remove()
    multi_kmz_mode_var.trace_add("write", update_multi_kmz_ref_visibility)
    update_multi_kmz_ref_visibility()
    ttk.Checkbutton(multi_kmz_tab, text="Include track rows with exactly 4 satellites (5+ recommended; GPS altitude may freeze or jump)", variable=multi_kmz_four_sat_var).grid(row=6, column=0, sticky="w", padx=8, pady=4)

    multi_kmz_comp_box = ttk.LabelFrame(multi_kmz_tab, text="After checking the KMZ files in Google Earth")
    multi_kmz_comp_box.grid(row=7, column=0, sticky="ew", padx=8, pady=6)
    multi_kmz_comp_box.columnconfigure(1, weight=1)
    make_wrapped_label(multi_kmz_comp_box, "Generate another compensated KMZ beside each CSV if the tracks need a small visual altitude adjustment.").grid(row=0, column=0, columnspan=3, sticky="ew", padx=4, pady=2)
    multi_kmz_comp_var = tk.StringVar(value="")
    ttk.Label(multi_kmz_comp_box, text="Compensation (m)").grid(row=1, column=0, sticky="w", padx=4, pady=2)
    ttk.Entry(multi_kmz_comp_box, textvariable=multi_kmz_comp_var, width=14).grid(row=1, column=1, sticky="w", padx=4, pady=2)

    multi_kmz_last: Dict[str, Any] = {"csvs": []}
    def _multi_kmz_base_options(offset: float = 0.0) -> KMZAltitudeOptions:
        save_parameter_settings(collect_dash_settings())
        return _kmz_elevation_options_from_vars(multi_kmz_mode_var, multi_kmz_manual_var, multi_kmz_four_sat_var, multi_kmz_confirm_var, offset=offset)
    def generate_multi_kmz(offset: float = 0.0, compensated: bool = False) -> None:
        try:
            csvs = multi_kmz_last.get("csvs") if compensated and multi_kmz_last.get("csvs") else _kmz_files_from_text(multi_kmz_paths_text)
            if not csvs:
                raise ValueError("Add at least one CSV file or folder containing CSV files.")
            opts = _multi_kmz_base_options(offset=offset)
            suffix = kmz_safe_offset_suffix(offset) if compensated else ""
            made = 0
            log(f"3D KMZ batch: {len(csvs)} CSV file(s)")
            for i, csv_path in enumerate(csvs, start=1):
                log(f"[{i}/{len(csvs)}] {os.path.basename(csv_path)}")
                out = kmz_write_kmz(csv_path, opts, output_suffix=suffix, manual_fallback_callback=_kmz_manual_fallback_dialog, status_callback=log)
                if out:
                    remember_kmz_output(out)
                    made += 1
            multi_kmz_last["csvs"] = csvs
            log(f"Done. Created {made} KMZ file(s).")
        except Exception as exc:
            messagebox.showerror("3D KMZ batch export failed", str(exc))
    def generate_multi_kmz_initial() -> None:
        generate_multi_kmz(offset=0.0, compensated=False)
    def generate_multi_kmz_compensated() -> None:
        try:
            offset = float(str(multi_kmz_comp_var.get()).strip())
        except Exception:
            messagebox.showerror("Invalid compensation", "Type an altitude compensation in metres, such as 5, 12.5, or -3.")
            return
        generate_multi_kmz(offset=offset, compensated=True)
    multi_kmz_buttons = ttk.Frame(multi_kmz_tab)
    multi_kmz_buttons.grid(row=8, column=0, sticky="ew", padx=8, pady=6)
    ttk.Button(multi_kmz_buttons, text="Move session KMZs to Recycle Bin", command=recycle_session_kmz_outputs).pack(side="left", padx=(0, 8))
    ttk.Button(multi_kmz_buttons, text="Generate compensated KMZs", command=generate_multi_kmz_compensated).pack(side="right", padx=(8, 0))
    ttk.Button(multi_kmz_buttons, text="Generate initial KMZs", command=generate_multi_kmz_initial).pack(side="right", padx=(8, 0))

    # ------------------------------------------------------------------
    # Tab 8: presets/help
    # ------------------------------------------------------------------
    presets_tab = ttk.Frame(notebook)
    notebook.add(presets_tab, text="Presets / notes")
    presets_tab.columnconfigure(0, weight=1)
    presets_tab.rowconfigure(1, weight=1)
    ttk.Label(presets_tab, text="How to use Flight Map Tools", font=("Segoe UI", 12, "bold")).grid(row=0, column=0, sticky="w", padx=8, pady=(8, 4))
    notes_frame, notes = make_scrolled_text(presets_tab, height=18, wrap="word")
    notes_frame.grid(row=1, column=0, sticky="nsew", padx=8, pady=6)
    notes.insert("end", f"Preset JSON location:\n{get_presets_json_path()}\n\nAircraft grouping JSON location:\n{get_aircraft_groups_json_path()}\n\n")
    notes.insert("end", "What this app can do:\n")
    notes.insert("end", "1. Process one Betaflight or ArduPilot-style EdgeTX CSV into an interactive HTML map.\n")
    notes.insert("end", "2. Process a whole folder of CSV logs recursively and save the HTML files beside each CSV.\n")
    notes.insert("end", "3. Analyse any supported raw or computed parameter in a polished interactive HTML report with a switchable-layer map, parameter-coloured route, clickable flagged episodes, inspection points, deterministic findings, and exportable Plotly timelines.\n")
    notes.insert("end", "4. Enrich one CSV or a recursive folder for Dashware or similar video-overlay software with selected heading/course, cardinal direction, coordinate speed, distances, MSL/AGL altitude, terrain elevation, logged/derived vertical speed, temperature, body angular rates, signal, power, energy, efficiency, and other selected columns. Optional GPX output is also available.\n")
    notes.insert("end", "5. Build an all-flights summary across folders/files, group renamed aircraft, split by year/month, and save the report as TXT.\n")
    notes.insert("end", "6. Create 3D KMZ maps for Google Earth with automatic or manual takeoff elevation and optional altitude compensation.\n")
    notes.insert("end", "7. Save custom presets in the JSON file beside the EXE so you can reuse map/stats/privacy/GPS-threshold settings.\n\n")
    notes.insert("end", "Basic steps for 2D map export:\n")
    notes.insert("end", "- Pick the Single CSV or Recursive tab.\n")
    notes.insert("end", "- Browse to a CSV/folder or paste a file path. Windows Copy as path text with quotes is accepted.\n")
    notes.insert("end", "- Use Built-in preset for the fast default output, Saved preset for your JSON presets, or Customize below to reveal all detailed options.\n")
    notes.insert("end", "- When customizing, choose map layer style, stats groups, privacy trimming, optional stats-line removal, and whether to include 4-satellite GPS sections.\n")
    notes.insert("end", "- Five satellites remains the recommended normal threshold. When a log contains 4-satellite rows, generated HTML and the status output warn whether those rows were included or excluded, because distance coverage, position, and especially GPS altitude can be less reliable.\n")
    notes.insert("end", "- To remove individual stats lines in Customize mode, click Preview/update stats line checklist and uncheck the lines you do not want. Recursive mode preserves different firmware/altitude layouts using stable line keys.\n")
    notes.insert("end", "- CSV column order is not assumed. Duplicate telemetry names are content-scored so a populated sensor can win over a blank/constant placeholder.\n")
    notes.insert("end", "- Newer CRSF GPS UTC date-time fields are kept separate from the ordinary local EdgeTX Date + Time used for flight timing; GPX uses the UTC field when available.\n")
    notes.insert("end", "- Flight-mode strings are used to distinguish ArduPilot, Betaflight and INAV where possible. Logs with at least 10% controller-managed throttle/navigation time are marked semi-autonomous so RC throttle is not mistaken for actual motor/TECS output.\n")
    notes.insert("end", "- Click Generate in the bottom-right corner.\n\n")
    notes.insert("end", "Flight data analysis:\n")
    notes.insert("end", "- Choose one CSV, then choose any available raw or computed parameter as the main analysis. The GUI changes its interpretation and threshold labels for satellite counts, signal quality, efficiency, speed, altitude, power, and other value types.\n")
    notes.insert("end", "- Set trusted/good, caution, and poor thresholds or low/medium/high bands. Choices are remembered per parameter in the existing preset/settings JSON.\n")
    notes.insert("end", "- The route, summary, findings, flagged-episode table, inspection points, and timeline all follow the selected parameter rather than being limited to GPS performance. Valid GPS below 4 satellites or missing GPS still breaks the mapped route.\n")
    notes.insert("end", "- Timeline time values use a true date/time axis with limited tick labels, avoiding the crowded overlapping x-axis text from v25. Choose any usable numeric original CSV columns or computed fields for time-axis comparison. The HTML has a dedicated exact-size PNG button plus Plotly's camera control; Full HD 1920 × 1080 is the default and can be changed in the GUI.\n")
    notes.insert("end", "- Converted relative altitude no longer develops artificial gaps when a Betaflight log rises above the old MSL-looking threshold; the initial pre-reset MSL sample is ignored only for the converted relative-altitude series.\n\n")
    notes.insert("end", "Dashware enrichment:\n")
    notes.insert("end", "- Choose one CSV or a folder recursively, select the extra columns, and optionally create GPX. Outputs are written beside each source CSV.\n")
    notes.insert("end", "- Elapsed time is always the first added column and follows the original Time column. Choose seconds, decimal minutes, or H:MM:SS.mmm. Original headers and rows are copied unchanged; only new columns are appended.\n")
    notes.insert("end", "- Heading and cardinal direction use one automatically selected source. Original Hdg is preferred when it covers at least 97% of moving rows and differs substantially from the centred 2-second GPS course in no more than 3% of comparisons; otherwise the better-covered source is used, with the other source filling isolated blanks.\n")
    notes.insert("end", "- Altitude MSL uses terrain at the first valid zero-relative-altitude GPS sample as the takeoff reference. Local terrain files are the default. Download compatible files from https://terrain.ardupilot.org/, keep names such as N49W114.DAT, and select any parent folder—even a broad folder such as Downloads. The app scans all subfolders recursively, opens and indexes every readable terrain file during the first terrain operation, accepts both legacy and current ArduPilot DAT files plus standard SRTM HGT files, prefers the finest matching grid automatically, and caches the fully loaded database until the app closes. ArduPilot DAT location is read from embedded metadata so renamed DAT files remain usable; standard HGT files must keep their coordinate filename because HGT contains no embedded location. Local mode queries every distinct logged GPS coordinate and only bilinearly interpolates within the terrain grid itself. OpenTopoData remains an optional online source/fallback and samples the route to respect public API limits. Negative generated AGL results are written as 0, while the original CSV remains unchanged.\n")
    notes.insert("end", "- Unit system, custom per-parameter units, decimal precision, elapsed format, angular-rate unit, joke altitude cap, and saved Dashware column selections use the existing preset/settings JSON. Decimal minutes always use two decimal places.\n")
    notes.insert("end", "- Ground-track turn rate is derived from the selected heading/course. Roll, pitch, and yaw rate are separate generated columns derived from the original attitude angles; radian/degree input is detected from the header and values, and output can be deg/s or rad/s.\n")
    notes.insert("end", "- GPX includes valid 4+ satellite track segments, elevation where available, timestamps, satellite count, and FPV extensions for speed and altitude.\n\n")
    notes.insert("end", "3D KMZ maps for Google Earth (KMZ code version 7):\n")
    notes.insert("end", "- Use Single 3D map for one CSV, or Multiple 3D maps for folders/files recursively.\n")
    notes.insert("end", "- Auto mode uses a genuine ASL/MSL CSV directly. Relative-altitude logs (including modern ArduPilot, Betaflight and INAV profiles) use the shared terrain source shown in every operational tab, then a compatible CSV takeoff reference if present, then manual entry if needed.\n")
    notes.insert("end", "- Five satellites is recommended. Both 3D tabs can optionally retain exactly 4-satellite rows for a more continuous route, with output warnings about less reliable position and GPS altitude.\n")
    notes.insert("end", "- If Google Earth terrain slightly hides the path, generate a compensated KMZ with a positive or negative metre offset.\n")
    notes.insert("end", "- The delete buttons only move KMZ files created in the current app session to the Recycle Bin and skip anything missing.\n\n")
    notes.insert("end", "All-flights summary:\n")
    notes.insert("end", "- Add one or more folders/files, then scan flights.\n")
    notes.insert("end", "- The app detects aircraft names from CSV model columns when present, otherwise from filenames.\n")
    notes.insert("end", "- Edit grouping lines like DetectedName = Final Aircraft Group to combine renamed aircraft. Delete unwanted lines to exclude those groups from the report.\n")
    notes.insert("end", "- Dates prefer the CSV Date column; filename date is used only as a fallback.\n")
    notes.insert("end", "- The skip checkbox filters out CSVs whose headers do not look like flight logs. Turn it off if you want to force-include every CSV and delete unwanted groups manually.\n")
    notes.insert("end", "- GPS-good time uses the selected 5- or 4-satellite threshold, controlled by the summary-tab checkbox. No/low-GPS time includes missing GPS, sats below the threshold, and large unlogged time gaps.\n\n")
    notes.insert("end", f"Credit: YouTube channel: Josh's Air (@joshthebuilder247)\n")
    notes.insert("end", f"Version: {APP_VERSION.replace('v', '')}\n")
    notes.configure(state="disabled")

    def open_preset_json() -> None:
        path = get_presets_json_path()
        if not os.path.isfile(path):
            save_user_presets(load_user_presets())
        try:
            if sys.platform.startswith("win"):
                os.startfile(path)  # type: ignore[attr-defined]
            else:
                messagebox.showinfo("Preset JSON path", path)
        except Exception as exc:
            messagebox.showerror("Open JSON failed", str(exc))

    ttk.Button(presets_tab, text="Open preset JSON in default editor", command=open_preset_json).grid(row=2, column=0, sticky="w", padx=8, pady=6)

    if initial_csvs:
        notebook.select(single_tab)
        root.after(600, lambda: single_options.refresh_throttle_channels(silent=True))
        root.after(700, lambda: refresh_analysis_metrics(silent=True))
        root.after(800, lambda: refresh_dash_throttle(silent=True))
    elif initial_folders:
        notebook.select(recursive_tab)
        root.after(600, lambda: recursive_options.refresh_throttle_channels(silent=True))
        root.after(700, lambda: refresh_dash_throttle(silent=True))
    else:
        root.after(600, lambda: single_options.refresh_throttle_channels(silent=True))
        root.after(700, lambda: refresh_analysis_metrics(silent=True))
        root.after(800, lambda: refresh_dash_throttle(silent=True))

    root.mainloop()
    return 0

def menu() -> None:
    print("\nFPV Flight Path Map — EdgeTX CSV ➜ HTML")
    print("=======================================")
    print("1) Process a single CSV file")
    print("2) Process ALL CSV files recursively in a folder")
    print("3) Single CSV interactive flight analysis report")
    print("4) All flights summary")
    print("5) Launch GUI")
    print("6) Exit\n")


def main_console() -> int:
    """Classic console workflow, retained for full advanced-feature access."""
    argv_paths = [p for p in sys.argv[1:] if p.lower().endswith(".csv")]

    if argv_paths:
        csv_path = normalize_path(argv_paths[0])
        run_options = choose_run_options(sample_csv_path=csv_path, preview_csv_paths=[csv_path])
        process_csv_to_html(csv_path, run_options)
        return 0

    while True:
        menu()
        choice = input("Choose (1-6): ").strip()

        if choice == "1":
            csv_in = normalize_path(input("CSV file path: "))
            if not os.path.isfile(csv_in):
                print("❌ File not found.")
                continue
            run_options = choose_run_options(sample_csv_path=csv_in, preview_csv_paths=[csv_in])
            process_csv_to_html(csv_in, run_options)

        elif choice == "2":
            folder = normalize_path(input("Folder path to scan: "))
            if not os.path.isdir(folder):
                print("❌ Folder not found.")
                continue

            csv_files = find_csv_files(folder)
            if not csv_files:
                print("⚠️  No .csv files found in that folder.")
                continue

            # Use the first CSV as a sample for menus that need to inspect available columns,
            # such as selectable throttle channel columns and ArduPilot-like altitude warnings.
            run_options = choose_run_options(sample_csv_path=csv_files[0], preview_csv_paths=csv_files)

            print(f"\nFound {len(csv_files)} CSV file(s). Processing...\n")
            made = 0
            for csv_path in csv_files:
                made += process_csv_to_html(csv_path, run_options)
            print(f"\n✅ Done. Created {made} HTML map file(s).\n")

        elif choice == "3":
            csv_in = normalize_path(input("CSV file path for data analysis: "))
            if not os.path.isfile(csv_in):
                print("❌ File not found.")
                continue
            process_single_csv_data_analysis(csv_in)

        elif choice == "4":
            process_all_flights_summary_cli()

        elif choice == "5":
            return launch_gui()

        elif choice == "6":
            print("Bye!")
            return 0
        else:
            print("❌ Invalid choice. Please enter 1, 2, 3, 4, 5, or 6.")


def main() -> int:
    """
    Start the full GUI by default.

    The old console workflow is still available with --cli for troubleshooting, but it is no longer
    required for normal use because the GUI exposes single/recursive maps, the interactive analysis report,
    Dashware/GPX enrichment, all-flights summary, 3D KMZ, presets, basemaps, stats, privacy, and line-removal options.
    """
    lowered = {arg.lower() for arg in sys.argv[1:]}
    if "--cli" in lowered or "/cli" in lowered or "-cli" in lowered:
        return main_console()
    return launch_gui()


if __name__ == "__main__":
    raise SystemExit(main())
