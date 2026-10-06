# Changelog

## v2.4 -- 2026-10-06
### Algo
- Loop closure: when a match is found all canonizations of the match are tested, not just the best one
  and and there is a second stage of consensus.
  Results differ from v2.3 (more loops closed).
- Tree Loading: added the vertical angle offset correction for kitti HDL-64, before voxelization. 0.205 deg as suggested in KISS-ICP. Performances on kitti substantially improved.
- Motion Model: onOriginReset is the single reset method when a new reference is chosen (reloc, new kd or loop).
- Voxelizer: enabled only when res is >0
- TreeLoader: added event for profiling
- TreeCPU_: Tree_CPUfromCPU_ copies all the fields.
- kd_ouster: new package for direct coupling with ouster lidars, and to record/play the raw datastream
	
### Build:
- all packages that have no ros dep are now plain CMake. srrg_make modules no loonger needed.
- a ros free build requires just colcon.
- ros parts confined in the _ros packages and loaded at runtime if compiled.
- binaries are in bin/ (no ros2 run)
- requires sync of srrg2_packages that  switched to plain CMake too


### Configs
- updated parameters. you need to recompute the trees now

## v2.3 -- 2026-09-18
### Algo
- Removed legacy stuff (imu full filter, gravity prior, localizer, useless tests)
- FlatLeafPolicy: added check for flatness on all eigenvalues
- Voxelizer: now uses flat structures, 3x speedup
- TreeLoader: enabled parallel tree construction.
	
## v2.2 -- 2026-07-16

### Algo
- Further separation of CPU and CUDA layers: now the cuda libraries override the compute conversion functions
on load, via explicit function pointers
- CMake auto-detects CUDA toolkit; HAVE_CUDA defaults ON but silently falls
  back to OFF if toolkit not found.
  __CUDACC__ replaces HAVE_CUDA as the C++ preprocessor guard in headers.
  HAVE_CUDA flag become local, and disappeared from the CPU path
- Tree_copyTo dispatch cleaned up: CPU-to-CPU constructor calls Tree_copyTo_
  directly; base-type constructor delegates through function pointer
- Apps are cuda agnostic, and attempt to load the cuda .so. If not found only the CPU types will be available.
  The .so should be in the LD_LIBRARY_PATH


### Build / infrastructure
- Docker: uses now a stratified build (base->srrg->cpu->cuda).
  CPU deployments stop at the CPU layer, GPU continue to the next stage.
  Colcon does not handle that for now.

	
## v2.1 -- 2026-07-09

### Algo
- ZeroVelMotionModel: new base class for the motion model hierarchy (zero-velocity prior)
- 2-step loop closure validation: ICP floating->match_kf, then ICP kf->match_kf
- Loop factors added based on inliers only (significantly improves CT-ICP results)
- Preliminary cure pass (map repair, experimental)
- Bug fix in map load (srrg2 solver: _last_graph_id + addFactor)
- Class decoupling: map_owner->optimizer_proc->bundler_proc->bundler and map_owner->tracker->optimizer->slam

### Config
- New proc chain entries for decoupled architecture
- Cure configuration added
- 2-step loop validation parameters
- Inlier-only factor policy

### Build / infrastructure
- kd_slam_setup.bash: single-file environment setup (KD_SLAM_* vars, PATH, dl.conf)
- make_cpu_confs.sh: generate CPU configs from CUDA ones
- README: Quick Guide added (tools, viewer controls, 6-step pipeline)
- kd_runme.sh, run_all.sh: hardcoded paths replaced with KD_SLAM_* env vars
- Docker: multi-stage Dockerfile (base/srrg/cpu/cuda targets), single build.sh, run.sh for interactive sessions
- Docker: kd_slam_setup.bash auto-sourced on startup, /data mount for datasets
- Docker: CUDA image requires host driver >= 560 and nvidia-container-toolkit
