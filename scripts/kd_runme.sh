#!/bin/bash
# kd_runme.sh -- run the KD-SLAM pipeline on a dataset
#
# Usage: kd_runme.sh convert
#        kd_runme.sh <variant> [run|eval|kittify]
#
# Variants: slam_icp  slam_cticp
#           slam_icp_ba  slam_cticp_ba
#           slam_icp_ba_ctba  slam_cticp_ba_ctba
#
# Dataset control (env vars):
#   DATASET   -- root of dataset (default: ~/kd_slam/vbr)
#   BAGS      -- bag directory   (default: $DATASET/bags)
#   GT        -- GT TUM files    (default: $DATASET/gt_files)
#   RESULTS   -- output root     (default: ~/kd_slam/results)
#   SEQUENCES -- space-separated sequence list
#
# Config control:
#   CONFIGS   -- config directory
#   SLAM_CONF -- override slam config (auto-selected by variant if unset)
#   BUNDLE_CONF -- override bundle config

set -e

VARIANT=${1:-slam_icp}
PHASE=${2:-run}

if [ "$VARIANT" = "convert" ]; then
  PHASE=convert
  VARIANT=slam_icp
fi

_WS=${KD_SLAM_ROS_WORKSPACE_INSTALL:-~/ws/install}
INSTALL=${INSTALL:-$_WS/kd_slam2}
CONFIGS=${CONFIGS:-${KD_SLAM_CONFIGS:-$(dirname "$0")/../kd_slam2/configs}}
_KD=${KD_SLAM_TEST:-~/kd_slam}
DATASET=${DATASET:-$_KD/vbr}
BAGS=${BAGS:-$DATASET/bags}
GT=${GT:-$DATASET/gt_files}
RESULTS=${RESULTS:-$_KD/results}

RUNNER=$INSTALL/bin/kd_slam
LOADER=$INSTALL/bin/kd_bundler
REPLAYER=$INSTALL/bin/kd_map_replay
CONVERTER=$INSTALL/bin/kd_converter
TRAJ_COMPARE=${TRAJ_COMPARE:-$_WS/kd_eval_tools/lib/kd_eval_tools/traj_compare}
TUM2POSES=${TUM2POSES:-$_WS/kd_eval_tools/lib/kd_eval_tools/kitti_tum2poses}

SEQUENCES=${SEQUENCES:-"diag_train0 diag_test0 colosseo_train0 colosseo_test0 pincio_train0 pincio_test0 spagna_train0 spagna_test0 ciampino_train0 ciampino_train1 ciampino_test0 ciampino_test1 campus_train0 campus_train1 campus_test0 campus_test1"}

# ---------- parse variant ----------
case $VARIANT in
  slam_icp)            SLAM_MODE=icp;   STAGE=slam ;;
  slam_cticp)          SLAM_MODE=cticp; STAGE=slam ;;
  slam_icp_ba)         SLAM_MODE=icp;   STAGE=ba   ;;
  slam_cticp_ba)       SLAM_MODE=cticp; STAGE=ba   ;;
  slam_icp_ba_ctba)    SLAM_MODE=icp;   STAGE=ctba ;;
  slam_cticp_ba_ctba)  SLAM_MODE=cticp; STAGE=ctba ;;
  *)
    echo "Unknown variant: $VARIANT"
    echo "Variants: slam_icp slam_cticp slam_icp_ba slam_cticp_ba slam_icp_ba_ctba slam_cticp_ba_ctba"
    echo "Phases:   run (default) | eval | kittify"
    exit 1 ;;
esac

[ "$PHASE" = "convert" ] && STAGE=convert

# configs (override with SLAM_CONF / BUNDLE_CONF env vars)
case $SLAM_MODE in
  icp)   SLAM_CONF_DRIVE=$CONFIGS/kd_slam_icp_drive.conf ;;
  cticp) SLAM_CONF_DRIVE=$CONFIGS/kd_slam_cticp_drive.conf ;;
esac
BUNDLE_CONF_DRIVE=${BUNDLE_CONF:-$CONFIGS/kd_bundle_drive.conf}
BUNDLE_CONF_HANDHELD=${BUNDLE_CONF:-$CONFIGS/kd_bundle_handheld.conf}

# ---------- map naming ----------
slam_map()  { echo "${RESULTS}/$1/$1_${SLAM_MODE}"; }
ba_map()    { echo "${RESULTS}/$1/$1_${SLAM_MODE}_ba"; }
ctba_map()  { echo "${RESULTS}/$1/$1_${SLAM_MODE}_ba_ctba"; }
kf_file()   { echo "${RESULTS}/$1/$1_${SLAM_MODE}.kf"; }
tum_file()  { echo "${RESULTS}/$1/$1_${VARIANT}.tum"; }

run_convert() {
  local seq=$1
  local bag=$BAGS/$seq
  local out=$BAGS/${seq}_tree
  [ -d "$bag" ] || { echo "SKIP $seq: bag not found at $bag"; return; }
  [ -d "$out" ] && { echo "SKIP $seq: $out already exists"; return; }
  local conf
  if [ -n "$SLAM_CONF" ]; then
    conf=$SLAM_CONF
  elif is_handheld $seq; then
    case $SLAM_MODE in
      icp)   conf=$CONFIGS/kd_slam_icp_handheld.conf ;;
      cticp) conf=$CONFIGS/kd_slam_cticp_handheld.conf ;;
    esac
  else
    conf=$SLAM_CONF_DRIVE
  fi
  echo "=== seq $seq: converting ==="
  $CONVERTER -c $conf -i $bag -o $out \
    2>&1 | tee $BAGS/${seq}_convert.log
}

delete_map() {
  rm -f "${1}_map.boss" "${1}_map.mcap"
  rm -rf "${1}_map_kd"
}

is_handheld() {
  case $1 in
    diag_*|colosseo_*|pincio_*|spagna_*) return 0 ;;
    *) return 1 ;;
  esac
}

run_slam() {
  local seq=$1
  local tree=$BAGS/${seq}_tree
  local bag=$BAGS/$seq
  local conf input
  if [ -d "$tree" ]; then
    input=$tree
  elif [ -d "$bag" ]; then
    input=$bag
  else
    echo "SKIP $seq: no tree or bag found"; return
  fi
  if [ -z "$SLAM_CONF" ]; then
    if is_handheld $seq; then
      case $SLAM_MODE in
        icp)   conf=$CONFIGS/kd_slam_icp_handheld.conf ;;
        cticp) conf=$CONFIGS/kd_slam_cticp_handheld.conf ;;
      esac
    else
      conf=$SLAM_CONF_DRIVE
    fi
  else
    conf=$SLAM_CONF
  fi

  local out=$RESULTS/$seq
  mkdir -p $out
  delete_map $(slam_map $seq)
  echo "=== seq $seq: slam $SLAM_MODE ==="
  /usr/bin/time -f "real %e user %U sys %S" \
    $RUNNER -c $conf -i $input \
      -om $(slam_map $seq) \
      -os $(kf_file $seq) \
      -ot $(tum_file $seq) \
    2>&1 | tee $out/${seq}_${VARIANT}.log
}

run_ba() {
  local seq=$1
  local in=$(slam_map $seq)
  local out=$(ba_map $seq)
  local bconf=$(is_handheld $seq && echo $BUNDLE_CONF_HANDHELD || echo $BUNDLE_CONF_DRIVE)
  if [ ! -f "${in}_map.boss" ]; then
    echo "SKIP $seq: ${in}_map.boss not found"; return
  fi
  delete_map $out
  echo "=== seq $seq: ICP bundle ==="
  $LOADER -c $bconf -im $in -om $out -b \
    2>&1 | tee ${RESULTS}/$seq/${seq}_${VARIANT}_ba.log
}

run_ctba() {
  local seq=$1
  local in=$(ba_map $seq)
  local out=$(ctba_map $seq)
  local bconf=$(is_handheld $seq && echo $BUNDLE_CONF_HANDHELD || echo $BUNDLE_CONF_DRIVE)
  if [ ! -f "${in}_map.boss" ]; then
    echo "SKIP $seq: ${in}_map.boss not found"; return
  fi
  delete_map $out
  echo "=== seq $seq: CT bundle ==="
  $LOADER -c $bconf -im $in -om $out -cb \
    2>&1 | tee ${RESULTS}/$seq/${seq}_${VARIANT}_ctba.log
}

run_replay() {
  local seq=$1
  local kf=$(kf_file $seq)
  local final_map
  case $STAGE in
    ba)   final_map=$(ba_map $seq) ;;
    ctba) final_map=$(ctba_map $seq) ;;
  esac
  if [ ! -f "$kf" ]; then
    echo "SKIP $seq: $kf not found"; return
  fi
  if [ ! -f "${final_map}_map.boss" ]; then
    echo "SKIP $seq: ${final_map}_map.boss not found"; return
  fi
  echo "=== seq $seq: replay -> $(tum_file $seq) ==="
  $REPLAYER -is $kf -im $final_map -ot $(tum_file $seq) \
    2>&1 | tee ${RESULTS}/$seq/${seq}_${VARIANT}_replay.log
}

process() {
  local seq=$1
  case $STAGE in
    convert) run_convert $seq ;;
    slam)    run_slam $seq ;;
    ba)      run_ba $seq; run_replay $seq ;;
    ctba)    run_ctba $seq; run_replay $seq ;;
  esac
}

# ---------- eval ----------
do_eval() {
  local eval_dir=$RESULTS/eval/$VARIANT
  mkdir -p "$eval_dir"
  local n=0
  for seq in $SEQUENCES; do
    local tum=$(tum_file $seq)
    local gt=$GT/${seq}_gt.tum
    if [ ! -f "$tum" ]; then
      echo "SKIP $seq: estimate not found"; continue
    fi
    if [ ! -f "$gt" ]; then
      echo "SKIP $seq: GT not found at $gt"; continue
    fi
    echo "=== $seq: eval ==="
    $TRAJ_COMPARE "$gt" "$tum" "$eval_dir" \
      2>&1 | tee "$eval_dir/${seq}_eval.log"
    ((n++)) || true
  done
  echo "Evaluated $n sequences -> $eval_dir"
  echo "=== Eval done ==="
}

# ---------- kittify (KITTI submission only) ----------
do_kittify() {
  local sub_dir=$RESULTS/submission/$VARIANT
  mkdir -p "$sub_dir"
  local n=0
  for seq in $SEQUENCES; do
    local tum=$(tum_file $seq)
    local calib=$DATASET/sequences/$seq/calib.txt
    local out=$sub_dir/${seq}.txt
    if [ ! -f "$tum" ]; then
      echo "SKIP $seq: estimate not found"; continue
    fi
    if [ ! -f "$calib" ]; then
      echo "SKIP $seq: calib not found (not a KITTI sequence?)"; continue
    fi
    echo "=== $seq: kittify ==="
    $TUM2POSES "$tum" "$out" "$calib"
    ((n++)) || true
  done
  echo "Kittified $n sequences -> $sub_dir"
  echo "=== Kittify done ==="
}

# ---------- main ----------
case $PHASE in
  eval)    do_eval;    exit 0 ;;
  kittify) do_kittify; exit 0 ;;
esac

for seq in $SEQUENCES; do
  process $seq
done

echo "=== ALL DONE ($VARIANT) ==="
