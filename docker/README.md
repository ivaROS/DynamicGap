# IL Docker image

`ghcr.io/ivaros/il-arena` (package of the ivaROS org, linked to ivaROS/DynamicGap; an org owner sets its visibility): the imitation-learning workspace of the lab PC, built,
for students on any PC. Install steps for students (Windows): the *Imitation Learning on Docker
(Windows) — Install Guide*. This file is for whoever maintains the image.

## What is in it

| Path | Content |
| --- | --- |
| `/root/arena_ws` | catkin workspace, built (no build type, as on the lab PC: Release makes pedsim_simulator abort) |
| `src/planners/dynamic_gap`, `src/arena/{arena-rosnav,evaluation,simulation-setup}` | branch `imitation_learning`, `origin` = ivaROS |
| other repos in `src` | pinned to the lab PC's commits (`il.repos`) |
| `/root/il_data/{datasets,models}` | the IL datasets and trained models, copied from the lab PC's `~/il_data` at build time |
| `/root/il_data/il_gap_selection` | sessions; students mount the volume `il_sessions` here |
| `/root/run_il_dataset.sh` | collection run; a link to the repo's `docker/run_il_dataset.sh`, so `git pull` updates it. After the run it checks each session and asks whether to upload it |
| `il_check_install` | checks display, OpenGL, branches, build, data, git, Drive |

ROS Noetic desktop-full, the lab PC's Python versions, CPU torch 1.12.1, rclone 1.68.2. RViz uses the
NVIDIA driver when the container runs with `--gpus all`, software OpenGL otherwise (`bashrc_il`).

## Code updates do not need a new image

Students `git pull` and `catkin build`. Rebuild the image when dependencies, pinned repos or the
dataset change, or so that new students start from newer code.

## Build, test, publish (lab PC)

```
docker/build_image.sh                      # ghcr.io/ivaros/il-arena:latest
docker run --rm --gpus all -e DISPLAY=:0 -v /tmp/.X11-unix:/tmp/.X11-unix \
    ghcr.io/ivaros/il-arena:latest il_check_install          # needs xhost +SI:localuser:root
docker run --rm -e DISPLAY=:0 -v /tmp/.X11-unix:/tmp/.X11-unix \
    ghcr.io/ivaros/il-arena:latest il_check_install          # software OpenGL, as on Windows
docker tag ghcr.io/ivaros/il-arena:latest ghcr.io/ivaros/il-arena:$(date +%F)
gh auth token | docker login ghcr.io -u <your GitHub user> --password-stdin   # token needs write:packages; ivaROS member
docker push ghcr.io/ivaros/il-arena:latest
docker push ghcr.io/ivaros/il-arena:$(date +%F)
```

`build_image.sh` clones the four IL repos from ivaROS (`IL_SOURCE=https://github.com/<org>` for a
staging copy; `origin` is ivaROS either way) and re-runs the clone whenever a branch moved. The
date tags are the fallback if a new `latest` breaks. A full build takes about 20 minutes; the
image is a 4.6 GB download. Docker on the lab PC: Docker CE + NVIDIA container toolkit, data in
`/local/docker`.

## Branches

Work is staged on the Udog-ILLINOIS forks (branch `imitation_learning`) and merged by pull request
into ivaROS `imitation_learning`. Students push to ivaROS (branch + pull request into
`imitation_learning`).

## Drive uploads

`il_tools/il_drive.py` (`il_upload_sessions`, `il_upload_eval`, and `il_eval_model` at its end)
copies runs to Google Drive `Research/ImitationLearning/ilData/<operator>/` with the same layout as
`~/il_data` (`il_gap_selection/<session>/`, `evals/<eval>/<model>/`). Sessions are copied, never
deleted; eval videos are deleted locally once their Drive copy's MD5 matches. The rclone remote is
`ildrive:`; `IL_DRIVE_REMOTE` and `IL_OPERATOR` select it and the person.
- Lab PC: remote `gdrive` (scope `drive.file`, `root_folder_id` = ilData) and the alias
  `ildrive: -> gdrive:`; `~/.bashrc` sets `IL_OPERATOR=ewan IL_DRIVE_REMOTE=ildrive:`.
- Students: their own token (`rclone authorize "drive"` on Windows), remote `ildrive` with
  `root_folder_id` = ilData, as editors of ilData (install guide Part 7).

Files that exist only on the lab PC, not in a repo, are not in the image (for example the
`gru_learn_2` world and `0_0ped_middle_start_v6.json`), so the suites `il_0ped_v6.yaml` and
`50ped_daniel_2.yaml` do not run in it; `il_dataset_small_v1.yaml` does.
