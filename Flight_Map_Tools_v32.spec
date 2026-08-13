# -*- mode: python ; coding: utf-8 -*-

# Windows PyInstaller build specification for Flight Map Tools v32.
# Build with:
#   py -3 -m PyInstaller --clean --noconfirm Flight_Map_Tools_v32.spec

a = Analysis(
    ['Flight_Map_Tools_v32.py'],
    pathex=[],
    binaries=[],
    datas=[],
    hiddenimports=[],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='Flight_Map_Tools_v32',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=['fpv_flight_tools_maple_leaf_v32.ico'],
    manifest='fpv_flight_tools_dpi_aware_v32.manifest',
)
