# Flight Map Tools v32

A desktop FPV telemetry toolkit for working with **EdgeTX / CRSF CSV flight logs**. It can turn telemetry logs into interactive 2D maps and analysis reports, enrich logs for video overlays, summarize many flights, and create 3D KMZ tracks for Google Earth.

Maintained by **Josh's Air** on YouTube **[@joshthebuilder247](https://www.youtube.com/@joshthebuilder247)**.

> **Windows users:** the easiest option is to download the latest compiled `.exe` from this repository's **Releases** section. No Python installation is needed for the compiled Windows build.

## What it can do

- **Process single CSV** — create an interactive HTML flight map from one telemetry log.
- **Process CSVs recursively** — process a whole folder tree and save each output beside its source CSV.
- **Flight data analysis** — analyse raw or computed telemetry parameters with an interactive map, coloured route, flagged episodes, inspection points, findings, and Plotly timelines that can be exported as PNG.
- **Dashware** — append selected computed telemetry columns for Dashware or similar video-overlay software, with optional GPX export.
- **All flights summary** — scan many logs, group renamed aircraft, filter/split by date, and save a TXT report.
- **Single / Multiple 3D maps** — create KMZ flight tracks for Google Earth with terrain/takeoff-elevation handling and optional altitude compensation.
- **Presets / notes** — save reusable map, stats, privacy, unit, terrain, and analysis settings beside the program.

The program understands the telemetry patterns it was built around for Betaflight, ArduPilot and INAV-style EdgeTX/CRSF logs. It also handles missing GPS data, satellite-count track breaks, duplicate telemetry points, relative/absolute altitude cases, RSSI diversity data, privacy trimming, and optional 4-satellite track inclusion with warnings.

## Download for Windows

1. Open the repository's **Releases** section.
2. Open the newest release, for example **v32**.
3. Download `Flight_Map_Tools_v32.exe` from the release assets.
4. Run the EXE.

## Run from Python source

The app is mostly Python standard-library code and uses Tkinter for the GUI. Two small packages are optional but useful:

- `tkinterdnd2` — enables GUI drag-and-drop where supported.
- `send2trash` — provides safe Trash/Recycle Bin behavior on non-Windows systems.

### Windows PowerShell

```powershell
# 1) Download or clone this repository, then open PowerShell in that folder.

# 2) Optional: install the helper packages.
py -3 -m pip install -r requirements-build.txt

# 3) Run the Python version directly.
py -3 Flight_Map_Tools_v32.py
```

If `py -3` is not recognized but `python` works on your computer, use `python` instead.

### macOS

```bash
cd /path/to/Flight-Map-Tools
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements-build.txt
python3 Flight_Map_Tools_v32.py
```

The official Python installer for macOS includes Tkinter. If the GUI cannot start, first confirm that your Python installation has Tk/Tkinter support.

### Linux

```bash
cd /path/to/Flight-Map-Tools
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements-build.txt
python3 Flight_Map_Tools_v32.py
```

On some Linux distributions Tkinter is packaged separately by the operating system. If `python3 -m tkinter` does not open a small test window, install your distribution's Tkinter/Tk package before running the app.

## Build your own executable/app

See **[BUILDING.md](BUILDING.md)** for simple Windows PowerShell, macOS, and Linux build instructions.

Important: PyInstaller is **not a cross-compiler**. Build the Windows EXE on Windows, the macOS app on macOS, and the Linux executable on Linux.

## Internet / offline notes

The desktop program itself does not require the internet for normal local CSV processing and HTML creation. However:

- Generated HTML maps use online map tiles and web libraries, so opening the finished interactive maps normally requires internet access.
- OpenTopoData terrain lookup is an optional online terrain source.
- Local ArduPilot `.DAT` / SRTM `.HGT` terrain files can be used for terrain-dependent operations without using the online terrain service.

## Privacy

Flight logs can contain exact home, launch, or flying locations. Before sharing a CSV, HTML, GPX, KMZ, screenshot, or screen recording publicly:

- Use the app's privacy trimming where appropriate.
- Review the finished output for recognizable launch/home locations.
- Do not commit personal flight logs to this repository unless they are deliberately sanitized.

The included `.gitignore` intentionally ignores common generated flight files and CSV logs to reduce accidental location-data uploads.

## Contributing / bug fixes

Bug reports and proposed fixes are welcome. See **[CONTRIBUTING.md](CONTRIBUTING.md)**.

The public repository is controlled by the maintainer. Other people can fork the project and propose changes with a **Pull Request**, but a proposed change does **not** become part of the official repository or official release unless the maintainer reviews and merges it.

## Releases and updates

The intended update flow is:

1. Update `Flight_Map_Tools_v32.py` (or the next versioned filename).
2. Test the source version and build locally.
3. Commit the source/docs changes to GitHub.
4. Create a new GitHub Release with a version tag such as `v33`.
5. Attach the compiled Windows EXE to that release.
6. Add short release notes describing the changes.

See **[RELEASE_CHECKLIST.md](RELEASE_CHECKLIST.md)** for the repeatable checklist.

## License

This repository is provided under the **MIT License**. See [LICENSE](LICENSE).

The MIT license allows others to use, modify, and redistribute the code while keeping the copyright/license notice. It does **not** give anyone permission to push changes into this GitHub repository or publish an official release here; repository permissions are controlled separately by GitHub.

## Disclaimer

This project processes telemetry and produces visualization/analysis outputs. Verify important flight, altitude, terrain, GPS, or safety-related conclusions independently before relying on them for real-world operational decisions.
