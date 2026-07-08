#!/bin/bash
# run_all.sh -- run slam_icp/cticp and _ba variants on VBR and KITTI

set -e

SCRIPTS=$(cd "$(dirname "$0")" && pwd)
RUNME=$SCRIPTS/kd_runme.sh
CONFIGS=${CONFIGS:-${KD_SLAM_CONFIGS:-$SCRIPTS/../kd_slam2/configs}}
_KD=${KD_SLAM_TEST:-$HOME/kd_slam}

VBR_SEQS_HH="diag_train0 diag_test0 colosseo_train0 colosseo_test0 pincio_train0 pincio_test0 spagna_train0 spagna_test0"
VBR_SEQS_DR="ciampino_train0 ciampino_train1 ciampino_test0 ciampino_test1 campus_train0 campus_train1 campus_test0 campus_test1"
VBR_SEQS="${VBR_SEQS_HH} ${VBR_SEQS_DR}"

KITTI_SEQS="00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21"
KITTI_GT_SEQS="00 01 02 03 04 05 06 07 08 09 10"
KITTI_GT_DIR=${KITTI_GT_DIR:-$_KD/kitti/gt_files}
KITTI_RESULTS=${KITTI_RESULTS:-$_KD/results}
TRAJ_COMPARE=${TRAJ_COMPARE:-${KD_SLAM_ROS_WORKSPACE_INSTALL:-$HOME/ws/install}/kd_eval_tools/lib/kd_eval_tools/traj_compare}

run_vbr() {
    local variant=$1
    echo "=== VBR: $variant ==="
    DATASET=$_KD/vbr \
    RESULTS=$_KD/results \
    SEQUENCES="$VBR_SEQS" \
        $RUNME $variant run
}

run_kitti() {
    local variant=$1
    local slam_mode=$2
    echo "=== KITTI: $variant ==="
    DATASET=$_KD/kitti \
    BAGS=$_KD/kitti/bags \
    RESULTS=$_KD/results \
    SEQUENCES="$KITTI_SEQS" \
    SLAM_CONF=$CONFIGS/kd_slam_${slam_mode}_drive_kitti.conf \
    BUNDLE_CONF=$CONFIGS/kd_bundle_kitti.conf \
        $RUNME $variant run
}

# VBR -- slam first, then ba
run_vbr slam_icp
run_vbr slam_cticp
run_vbr slam_icp_ba
run_vbr slam_cticp_ba

# VBR eval
for variant in slam_icp slam_cticp slam_icp_ba slam_cticp_ba; do
    echo "=== VBR eval: $variant ==="
    DATASET=$_KD/vbr \
    RESULTS=$_KD/results \
    SEQUENCES="$VBR_SEQS" \
        $RUNME $variant eval
done

# KITTI -- ICP only (pre-deskewed)
run_kitti slam_icp    icp
run_kitti slam_icp_ba icp

# KITTI eval
for variant in slam_icp slam_icp_ba; do
    echo "=== KITTI eval: $variant ==="
    DATASET=$_KD/kitti \
    BAGS=$_KD/kitti/bags \
    RESULTS=$_KD/results \
    GT=$KITTI_GT_DIR \
    SEQUENCES="$KITTI_GT_SEQS" \
    SLAM_CONF=$CONFIGS/kd_slam_icp_drive_kitti.conf \
        $RUNME $variant eval
done

echo "=== ALL DONE ==="
