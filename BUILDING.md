# Building Flight Map Tools

These instructions are for people who want to build the program themselves instead of downloading a release.

## Before you start

- Download/clone the whole repository, not just the `.py` file.
- Keep `Flight_Map_Tools_v32.py`, the `.spec`, `.manifest`, and `.ico` together for the Windows build.
- PyInstaller builds for the operating system it is running on. It does not create a Windows EXE from Linux/macOS or a macOS app from Windows.

## Windows 10/11 — PowerShell

Open PowerShell in the repository folder.

```powershell
# Check Python.
py -3 --version

# Install/upgrade the build tools and optional GUI helpers.
py -3 -m pip install --upgrade pip
py -3 -m pip install -r requirements-build.txt

# Build using the supplied Windows spec file.
py -3 -m PyInstaller --clean --noconfirm Flight_Map_Tools_v32.spec
```

The finished file should be:

```text
dist\Flight_Map_Tools_v32.exe
```

Run it from PowerShell for the first test:

```powershell
.\dist\Flight_Map_Tools_v32.exe
```

If `py -3` is unavailable, replace it with the Python command that works on your system, commonly `python`.

### Rebuilding after code changes

Delete old build output if you want a completely fresh build, or simply use `--clean` as shown above.

The supplied Windows `.spec` already points to:

- `Flight_Map_Tools_v32.py`
- `fpv_flight_tools_maple_leaf_v32.ico`
- `fpv_flight_tools_dpi_aware_v32.manifest`
- a windowed/no-console build named `Flight_Map_Tools_v32`

## macOS — build an app locally

Install a current Python 3 for macOS, then in Terminal:

```bash
cd /path/to/Flight-Map-Tools
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements-build.txt
python3 -m PyInstaller --clean --noconfirm --onefile --windowed --name Flight_Map_Tools_v32 Flight_Map_Tools_v32.py
```

PyInstaller will place its output in `dist/`; `--windowed` on macOS also creates an application bundle. For a polished public macOS release, code signing/notarization is a separate platform-specific distribution step.

## Linux — build locally

In a terminal:

```bash
cd /path/to/Flight-Map-Tools
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements-build.txt
python3 -m PyInstaller --clean --noconfirm --onefile --name Flight_Map_Tools_v32 Flight_Map_Tools_v32.py
```

Output will be under `dist/`.

If Tkinter is not installed by your Linux distribution, install the distribution's Tk/Tkinter package first. You can test Tkinter with:

```bash
python3 -m tkinter
```

## Optional packages

`requirements-build.txt` includes:

- `pyinstaller` — creates distributable executables/apps.
- `tkinterdnd2` — optional drag-and-drop support.
- `send2trash` — optional safe trash behavior on non-Windows systems.

The main program is otherwise built primarily from Python's standard library and Tkinter.

## What to upload to GitHub

Commit the source and documentation to the repository. Put compiled binaries such as `Flight_Map_Tools_v32.exe` in a **GitHub Release** rather than committing them to the normal source tree.
