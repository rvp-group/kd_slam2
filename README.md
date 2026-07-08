# KD-SLAM

One KD-Tree to deskew them all, one KD-Tree to match them,
one KD-Tree to close the loops and in the map bind them.

[paper (PDF)] (https://github.com/rvp-group/kd_slam2/blob/main/paper/kd_slam.pdf)

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.21095943.svg)](https://doi.org/10.5281/zenodo.21095943)

[Video]       (https://youtu.be/c-sCCt9hMmI)

---

## Build (Docker -- easiest)

```bash
cd docker
./build.sh          # CPU only
./build.sh --cuda   # with CUDA
```

Produces image `kd_slam2:cpu` or `kd_slam2:cuda`.

---

## Build (native, ROS2 Jazzy)

**System dependencies**

```bash
apt install libeigen3-dev libopencv-dev libsuitesparse-dev \
            libglfw3-dev libgl-dev freeglut3-dev \
            libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
            liblz4-dev libzstd-dev libqglviewer-dev-qt5 python3-vcstool
```

**Workspace setup**

```bash
mkdir -p ~/ws/src
cd ~/ws/src
git clone https://github.com/rvp-group/kd_slam2
./kd_slam2/scripts/srrg_pull.sh
```

**Build**

```bash
cd ~/ws
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DHAVE_CUDA=0
```

Set `-DHAVE_CUDA=1` for CUDA support.

The default configs use CUDA ICP/CT-ICP types. If building without CUDA, generate
CPU variants with:

```bash
kd_slam2/configs/make_cpu_confs.sh
```

This produces `*_cpu.conf` copies with `ICPCPU3D`/`CTICPCPU3D` in place of the CUDA types.


---

## Quick Guide

### Tools

| binary | purpose |
|---|---|
| `srrg2_config_visualizer` | inspect and edit BOSS `.conf` parameter files |
| `kd_converter` | preprocess a raw bag into a tree bag (run once per sequence) |
| `kd_slam` | run SLAM on a bag or a tree bag; outputs map, keyframes, and TUM trajectory |
| `kd_bundler` | load a map, run bundle adjustment, write a refined map |
| `kd_map_replay` | replay keyframes through a map to extract a final TUM trajectory |
| `traj_compare` | compute ATE/RPE against a ground-truth TUM file |

All binaries are on PATH after sourcing `kd_slam_setup.bash`.
Use `-h` to list command line parameters.

### Config visualizer

`srrg2_config_visualizer` is a GUI tool for inspecting and editing BOSS config
files (the `.conf` files in `kd_slam2/configs/`).  It needs to know where your
workspace libraries are installed via `kd_slam2/configs/dl.conf`.

The `so_paths` entry in `dl.conf` uses the env var `KD_SLAM_ROS_WORKSPACE_INSTALL`,
set by `scripts/kd_slam_setup.bash` (see Quick start).

The visualizer crawls from the current directory up to the filesystem root
looking for `dl.conf`. `kd_slam_setup.bash` copies it to `~/dl.conf` automatically
on first source. Then launch the visualizer:

```bash
ros2 run srrg2_config_visualizer srrg2_config_visualizer \
    -c kd_slam2/configs/kd_slam_icp_drive.conf
```

Replace the last argument with whichever config you want to inspect.
The visualizer lets you browse the full parameter tree, edit values,
and save back to the file without hand-editing BOSS JSON.

### Viewer


Pass `-V 2` to `kd_slam` or `kd_bundler` to open the 3D viewer.
Data starts paused; press **Space** to begin.

| key | action |
|---|---|
| `Space` | pause / resume |
| `S` | toggle step mode (advance one frame at a time) |
| `F` | toggle follow-robot camera |
| `H` | toggle HUD overlay |
| `L` | toggle log overlay |
| `B` | *(bundler)* run rigid ICP bundle adjustment |
| `C` | *(bundler)* run CT-ICP bundle adjustment |
| `G` | *(bundler)* run cure -- experimental, see note below |

### Quick start

**1. Set up the environment**

Copy the setup file to your home directory and edit the two variables at the top:

```bash
cp /path/to/kd_slam2/scripts/kd_slam_setup.bash ~/kd_slam_setup.bash
# edit KD_SLAM_ROS_WORKSPACE and KD_SLAM_TEST at the top of ~/kd_slam_setup.bash
source ~/kd_slam_setup.bash
# optionally add to ~/.bashrc if you use kd_slam regularly
```


**2. Download a test sequence**

> *Pre-computed tree bags and a short test sequence are available here: https://drive.google.com/drive/folders/1LgFxgOsP95HbQVAJUiexE2pzaCtMVfzA?usp=drive_link
> Download and unpack so that `$KD_SLAM_TEST/vbr/bags/<seq>/` contains the tree bag
> and `$KD_SLAM_TEST/vbr/gt_files/<seq>_gt.tum` contains the ground truth.

**3. Run SLAM**

```bash
kd_slam \
    -c $KD_SLAM_CONFIGS/kd_slam_icp_handheld.conf \
    -i $KD_SLAM_TEST/vbr/bags/<seq>_tree \
    -om $KD_SLAM_TEST/results/<seq>/<seq>_icp \
    -os $KD_SLAM_TEST/results/<seq>/<seq>_icp.kf \
    -ot $KD_SLAM_TEST/results/<seq>/<seq>_icp.tum
```

If you use a drive dataset (ciampino, campus) use kd_slam_icp_drive.conf
Add `-V 2` to open the viewer.
Outputs: `<prefix>_map.boss` (map), `<prefix>.kf` (keyframes), `<prefix>.tum` (trajectory).

**4. Run bundle adjustment**

```bash
kd_bundler \
    -c $KD_SLAM_CONFIGS/kd_bundle_handheld.conf \
    -im $KD_SLAM_TEST/results/<seq>/<seq>_icp \
    -om $KD_SLAM_TEST/results/<seq>/<seq>_icp_ba \
    -b
```

Use `-cb` instead of `-b` for CT-ICP bundle adjustment.
Add `-V 2` to open the viewer (use `B`/`C` to trigger passes manually).

**5. Extract trajectory from the bundled map**

```bash
kd_map_replay \
    -is $KD_SLAM_TEST/results/<seq>/<seq>_icp.kf \
    -im $KD_SLAM_TEST/results/<seq>/<seq>_icp_ba \
    -ot $KD_SLAM_TEST/results/<seq>/<seq>_icp_ba.tum
```

**6. Evaluate**

```bash
traj_compare \
    $KD_SLAM_TEST/vbr/gt_files/<seq>_gt.tum \
    $KD_SLAM_TEST/results/<seq>/<seq>_icp_ba.tum \
    $KD_SLAM_TEST/results/eval/
```

Reports `ATE [R, T]` -- use the `T` (translation) metric; `R` is unreliable on
straight sequences due to the rotation null space along the travel axis.

> **Note on cure (`G`):** the map repair pass is functional but still under
> development.  Use it for exploration, not for benchmarking.

---

## Data layout

The scripts expect this folder structure under the dataset root
(default `$KD_SLAM_TEST/vbr`, override with `DATASET=`):

```
$KD_SLAM_TEST/
  vbr/
    bags/
      diag/          # raw ROS2 bag
      diag_tree/     # precomputed tree (generated by the convert phase)
      colosseo/
      colosseo_tree/
      ...
    gt_files/
      diag_gt.tum    # ground truth in TUM format
      colosseo_gt.tum
      ...
  kitti/
    dataset/
      sequences/
        00/  01/  ...   # standard KITTI layout (velodyne/, calib.txt, ...)
    gt_files/
      00_gt.tum
      ...
  results/           # output root; created automatically
```

VBR bags are distributed as ROS1 `.bag` chunks.  Before the first run,
merge and convert each sequence:

```bash
# requires: pip install rosbags
scripts/vbr_merge.sh diag     /path/to/vbr/diag_chunks/
scripts/vbr_merge.sh colosseo /path/to/vbr/colosseo_chunks/
# ... one call per sequence; output lands in $KD_SLAM_TEST/vbr/bags/<seq>/
```

Override `TOPICS` if your sensor uses different topic names
(default: `/ouster/points /ouster/imu`).

Precomputed trees are optional but recommended: they compress the bags
and remove the bag-reading bottleneck.
Generate them once with `scripts/kd_runme.sh slam_icp convert`.

---

## Run

```bash
source ~/kd_slam_setup.bash
scripts/kd_runme.sh slam_icp          # VBR (default)
scripts/kd_runme.sh slam_icp eval

# KITTI: one-time dataset prep
scripts/kitti2standard.sh
# then
DATASET=$KD_SLAM_TEST/kitti/dataset \
SEQUENCES="00 01 02 03 04 05 06 07 08 09 10" \
SLAM_CONF=$KD_SLAM_CONFIGS/kd_slam_icp_drive_kitti.conf \
scripts/kd_runme.sh slam_icp
```

VBR has two sensor types: OS1 (drive) and OS0 (handheld).
The default config is drive; override for handheld sequences (diag, colosseo, pincio, spagna):

```bash
SEQUENCES=spagna_train0 SLAM_CONF=$KD_SLAM_CONFIGS/kd_slam_icp_handheld.conf scripts/kd_runme.sh slam_icp
```

See `scripts/kd_runme.sh` for the full variant and phase list.

Videos: https://youtu.be/c-sCCt9hMmI
        https://youtu.be/aNCdJOZsXM0
        https://youtu.be/ANkpNj99f3A
