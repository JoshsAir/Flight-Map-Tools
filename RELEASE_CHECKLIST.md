# Release Checklist

Use this whenever you publish a new Flight Map Tools version.

- [ ] Start from the current reviewed `main` branch.
- [ ] Update the app version number and public filename if the version changes.
- [ ] Update the Windows `.spec` filename/name references for the new version.
- [ ] Run `python -m py_compile` (or equivalent) on the source.
- [ ] Open the GUI and test the major tabs you changed.
- [ ] Test at least one normal 2D HTML export.
- [ ] If changed, test analysis, Dashware/GPX, summary, and KMZ output as applicable.
- [ ] Build the Windows EXE on Windows with PyInstaller.
- [ ] Run the freshly built EXE on a trusted Windows machine.
- [ ] Scan the release binary with Windows Security/your normal antivirus.
- [ ] Review `git status` and make sure no personal CSV, HTML, GPX, KMZ, JSON preset, terrain, or private-location files are included.
- [ ] Commit and push the reviewed source/documentation changes.
- [ ] Create a GitHub Release tag such as `v33`.
- [ ] Attach the compiled EXE to the Release.
- [ ] Add concise release notes: what changed, notable fixes, and any known limitations.
- [ ] Publish the Release and verify the download works.
- [ ] Update screenshots/README only if the interface or outputs changed enough to make old images misleading.
