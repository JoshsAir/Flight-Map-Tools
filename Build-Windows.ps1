$ErrorActionPreference = "Stop"

Write-Host "Flight Map Tools v32 - Windows build helper"
Write-Host "Installing/updating build requirements..."

if (Get-Command py -ErrorAction SilentlyContinue) {
    py -3 -m pip install --upgrade pip
    py -3 -m pip install -r requirements-build.txt
    py -3 -m PyInstaller --clean --noconfirm Flight_Map_Tools_v32.spec
} elseif (Get-Command python -ErrorAction SilentlyContinue) {
    python -m pip install --upgrade pip
    python -m pip install -r requirements-build.txt
    python -m PyInstaller --clean --noconfirm Flight_Map_Tools_v32.spec
} else {
    throw "Python was not found. Install Python 3 first, then run this script again."
}

Write-Host ""
Write-Host "Build finished. Look for: dist\Flight_Map_Tools_v32.exe"
