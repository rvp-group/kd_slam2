# KD-SLAM

One KD-Tree to deskew them all, one KD-Tree to match them,
one KD-Tree to close the loops and in the map bind them.

[paper (PDF)](https://github.com/rvp-group/kd_slam2/blob/main/paper/kd_slam.pdf)

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.21095943.svg)](https://doi.org/10.5281/zenodo.21095943)

[Video](https://youtu.be/c-sCCt9hMmI)

---

## Build (native, ROS2 Jazzy -- recommended)

**System dependencies**

```bash
sudo apt install libeigen3-dev libopencv-dev libsuitesparse-dev \
            libglfw3-dev libgl-dev freeglut3-dev \
            libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
            liblz4-dev libzstd-dev libqglviewer-dev-qt5 python3-vcstool \
            python3-colcon-common-extensions \
            ros-jazzy-rosbag2-cpp \
            ros-jazzy-sensor-msgs \
            ros-jazzy-tf2 \
            ros-jazzy-tf2-msgs \
            ros-jazzy-pcl-conversions \
            ros-jazzy-pcl-ros \
            ros-jazzy-cv-bridge \
            ros-jazzy-image-transport \
            ros-jazzy-nav-msgs \
            ros-jazzy-std-msgs \
            ros-jazzy-rosbag2-transport \
            ros-jazzy-rosbag2-storage \
            ros-jazzy-rcl-interfaces 
```

**Workspace setup**

```bash
mkdir -p ~/ws/src
cd ~/ws/src
git clone https://github.com/rvp-group/kd_slam2
./kd_slam2/scripts/srrg_pull.sh
```

**Build (Native)**

```bash
cd ~/ws
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DHAVE_CUDA=OFF
```

Set `-DHAVE_CUDA=ON` for CUDA support.


---

## Build (native, no ros)

**System dependencies (no ros)**

```bash
sudo apt install libeigen3-dev libopencv-dev libsuitesparse-dev \
            libglfw3-dev libgl-dev freeglut3-dev \
            libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
            liblz4-dev libzstd-dev libqglviewer-dev-qt5 \
            vcstool colcon
```

**Workspace setup (no ros)**

```bash
mkdir -p ~/ws/src
cd ~/ws/src
git clone https://github.com/rvp-group/kd_slam2
./kd_slam2/scripts/srrg_pull.sh
```

**Build (Native, no ros)**

```bash
cd ~/ws
colcon build \
 --packages-skip kd_slam2_msgs srrg2_core_msgs \
   kd_slam2_ros srrg2_core_ros \
 --cmake-args -DHAVE_CUDA=OFF
```

Set `-DHAVE_CUDA=ON` for CUDA support.


---

## Build (Docker)

```bash
docker/build.sh --srrg   # shared base (build once)
docker/build.sh          # CPU image
docker/build.sh --cuda   # CUDA image
```

All images are based on `nvidia/cuda:12.6.0-devel-ubuntu24.04`.
The `--srrg` stage builds the shared srrg dependencies and is cached; rebuild
it only when `docker/repos.yml` changes.
The CPU image runs on any x86 machine.
The CUDA image requires the NVIDIA Container Toolkit on the host.
On docker the gui will be slower due to the lack of hardware acceleration.

**Run**

```bash
docker/run.sh          # CPU
docker/run.sh --cuda   # CUDA
```

Your data (`$KD_SLAM_TEST`) is mounted as `/data` inside the container.
Results written to `/data` land back on the host.
`kd_slam_setup.bash` is sourced automatically on startup.

The CUDA variant requires the NVIDIA Container Toolkit on the host (once per host):

```bash
sudo apt install nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

---

## Quick Guide

### Tools

| binary | purpose |
|---|---|
| `srrg2_config_visualizer` | aliased as confviz: visual IDE for BOSS pipelines: load shared libs, browse and instantiate configurables, wire up processing graphs, edit parameters, save configs  |
| `kd_converter` | preprocess a raw bag into a tree bag (run once per sequence) |
| `kd_slam` | run SLAM on a bag or a tree bag; outputs map, keyframes, and TUM trajectory |
| `kd_bundler` | load a map, run bundle adjustment, write a refined map |
| `kd_map_replay` | replay keyframes through a map to extract a final TUM trajectory |
| `traj_compare` | compute ATE/RPE against a ground-truth TUM file |
| `ouserver` | lightweight ouster server, can record logs in raw udp format |
| `ouread`   | reader for ouster logs |
| `kd_live`   | live player from ouster (either the raw udp dump or the connected device) |

All binaries are on PATH after sourcing `kd_slam_setup.bash`.
Use `-h` to list command line parameters.

### Dynamic Loading
The system operates with dynamic loading of objects listed in a `dl.conf`. The tools crawl
from the current folder to /, looking for this file. 
`kd_slam_setup.bash` installs a copy in your home, if not present.
 To use the ROS layer (reading/writing rosbags), `dl.conf` must list
 `kd_slam2_ros/lib/libkd_io_rosbag.so`. If your `~/dl.conf` predates v2.4, add that
 entry or delete the file and source the setup script again.

### Config visualizer

`srrg2_config_visualizer` (`confviz` alias after sourcing `kd_slam_setup.bash`)
is a visual IDE for the BOSS/srrg framework. It loads shared libraries at
runtime (listed in `dl.conf`), discovers all registered configurables, and lets
you compose processing pipelines graphically: instantiate modules, wire inputs
to outputs, set parameters, and save the result as a `.conf` file -- without
touching BOSS JSON by hand. Don't do that.
The configs in `kd_slam2/configs/` were built with
it and can be opened and modified from it.
After sourcing the setup script, launch it with:

```bash
confviz -c $KD_SLAM_CONFIGS/kd_slam_icp_drive.conf
```

### Viewer


Pass `-g` to `kd_slam` or `kd_bundler` to open the 3D viewer.
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

The quick start requires the ROS layer.

**1. Set up the environment**

Copy the setup file to your home directory and edit the two variables at the top:

```bash
cp /path/to/kd_slam2/scripts/kd_slam_setup.bash ~/kd_slam_setup.bash
# edit KD_SLAM_ROS_WORKSPACE and KD_SLAM_TEST at the top of ~/kd_slam_setup.bash
source ~/kd_slam_setup.bash
# optionally add to ~/.bashrc if you use kd_slam regularly
```


**2. Download a test sequence**

> *Pre-computed tree bags and a short test sequence are available here*: https://drive.google.com/drive/folders/1LgFxgOsP95HbQVAJUiexE2pzaCtMVfzA?usp=drive_link
> Download and unpack so that `$KD_SLAM_TEST/vbr/bags/<seq>/` contains the tree bag
> and `$KD_SLAM_TEST/vbr/gt_files/<seq>_gt.tum` contains the ground truth.

**3. Prepare the configs**
The default configs use CUDA ICP/CT-ICP types. If you want to run the pipeline on CPU, generate
the CPU variants with:

```bash
$KD_SLAM_CONFIGS/make_cpu_confs.sh
```
This produces `*_cpu.conf` copies with `ICPCPU3D`/`CTICPCPU3D` in place of the CUDA types.

Alternatively you can load the pipeline with confviz and rewire the types.

**4. Run SLAM**

```bash
kd_slam \
    -c $KD_SLAM_CONFIGS/kd_slam_icp_handheld.conf \
    -i $KD_SLAM_TEST/vbr/bags/<seq>_tree \
    -om $KD_SLAM_TEST/results/<seq>/<seq>_icp \
    -os $KD_SLAM_TEST/results/<seq>/<seq>_icp.kf \
    -ot $KD_SLAM_TEST/results/<seq>/<seq>_icp.tum
```

If you use a drive dataset (ciampino, campus) use kd_slam_icp_drive.conf
Add `-g` to open the viewer.
Outputs: `<prefix>_map.boss` (map), `<prefix>.kf` (keyframes), `<prefix>.tum` (trajectory).

**5. Run bundle adjustment**

```bash
kd_bundler \
    -c $KD_SLAM_CONFIGS/kd_bundle_handheld.conf \
    -im $KD_SLAM_TEST/results/<seq>/<seq>_icp \
    -om $KD_SLAM_TEST/results/<seq>/<seq>_icp_ba \
    -b
```

Use `-cb` instead of `-b` for CT-ICP bundle adjustment.
Add `-V 2` to open the viewer (use `B`/`C` to trigger passes manually).

**6. Extract trajectory from the bundled map**

```bash
kd_map_replay \
    -is $KD_SLAM_TEST/results/<seq>/<seq>_icp.kf \
    -im $KD_SLAM_TEST/results/<seq>/<seq>_icp_ba \
    -ot $KD_SLAM_TEST/results/<seq>/<seq>_icp_ba.tum
```

**7. Evaluate**

```bash
traj_compare \
    $KD_SLAM_TEST/vbr/gt_files/<seq>_gt.tum \
    $KD_SLAM_TEST/results/<seq>/<seq>_icp_ba.tum \
    $KD_SLAM_TEST/results/eval/
```

Reports 

 - `ATE_VBR [R, T]` -- mean error as used in the vbr benchmark
 - `ATE_RMSE [R, T]` -- root mean square error (classical, used in many papers)

For ATE use the `T` (translation) metric; `R` is unreliable on
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

Videos: 

- https://youtu.be/c-sCCt9hMmI
- https://youtu.be/aNCdJOZsXM0
- https://youtu.be/ANkpNj99f3A
