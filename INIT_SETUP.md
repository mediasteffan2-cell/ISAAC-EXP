# Repository Initialization Notes

This project was cleaned up so it can be published as a fresh repository.

## What was cleaned
- Removed all runtime log files under `logs/` and added the directory to `.gitignore` so future simulator/ROS runs do not pollute Git history.
- Left an empty `logs/.gitkeep` placeholder so scripts that expect the folder continue to work.
- Documented the exact Git bootstrap steps below so you can point the repo to a new remote such as `https://github.com/SteffanWolter/OPUS2-GO2-ISAAC`.

## How to create the new Git repo
1. From the project root run `rm -rf .git` if you want to discard the old history (optional).
2. Initialize and commit:
   ```bash
   git init
   git add .
   git commit -m "Initial commit"
   ```
3. Add the new remote and push:
   ```bash
   git branch -M main
   git remote add origin https://github.com/SteffanWolter/OPUS2-GO2-ISAAC.git
   git push -u origin main
   ```
4. Whenever you want to update GitHub, use the usual `git add/commit/push` commands.

These instructions mirror the quick-start block that GitHub shows after creating an empty repository, but tailored for this workspace.
