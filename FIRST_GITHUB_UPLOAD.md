# First GitHub Upload — Simple Steps

Your empty repository is already created correctly. The "Quick setup" screen simply means it does not have files yet.

## Easiest method: GitHub website

1. Unzip the `Flight-Map-Tools-GitHub-Ready.zip` package.
2. Open your GitHub repository.
3. On the empty repository page, click **uploading an existing file** (or **Add file → Upload files** later).
4. Drag the **contents** of the `Flight-Map-Tools-GitHub-Ready` folder into the upload area.
5. Make sure files such as `README.md`, `Flight_Map_Tools_v32.py`, `.gitignore`, `BUILDING.md`, and the `.github` folder are included.
6. Use a commit message such as `Initial public release files`.
7. Commit the files to `main`.
8. Refresh the repository page. GitHub will automatically render `README.md` as the main project description.

## Then publish the Windows EXE as a Release

Do not treat the EXE like normal source code.

1. Build or locate your trusted `Flight_Map_Tools_v32.exe`.
2. In the repository, open **Releases**.
3. Choose **Draft a new release**.
4. Create/select tag `v32`.
5. Release title: `Flight Map Tools v32`.
6. Add a short note such as `First public GitHub release of Flight Map Tools v32.`
7. Attach `Flight_Map_Tools_v32.exe` as a release asset.
8. Publish the release.
9. Test the release download yourself.

## Protect the official code

For a personal public repository, people who are not collaborators do not have write access to your repository. They can still fork it and propose changes.

For extra protection of `main`:

1. Open **Settings** in the repository.
2. Open **Rules → Rulesets** (or branch protection, depending on the GitHub interface shown to you).
3. Create a ruleset targeting the default branch `main`.
4. Turn on **Require a pull request before merging**.
5. Require at least **1 approval**.
6. Turn on **Require review from Code Owners** if that option is available.
7. Keep **Block force pushes** enabled.
8. Save/activate the ruleset.

The included `.github/CODEOWNERS` file names `@JoshsAir` as the code owner, so proposed changes are clearly routed to you for review.

## Do not upload these

- `all chats so far.txt`
- personal flight CSVs
- generated maps containing private locations
- JSON settings containing anything you do not want public
- build folders such as `build/` or `dist/`
- passwords, API keys, tokens, or private keys

The provided `.gitignore` helps prevent many of these from being committed accidentally when using Git from a computer.
