#!/usr/bin/env bash
# ======================================================================
# run_safety_motion_suite.sh — Policy + Motion anomaly end-to-end test
# Stack (patient side):
#   - patient_node
#   - safety_policy_manager
#   - patient_motion_anomaly_handler (ON by default; set WITH_DETECTOR=0 to disable)
#
# What this verifies:
#   - Motion anomalies → /safety/motion_event → SafetyPolicyManager sets mode=passive
#   - Policy returns to normal after hysteresis
#   - Patient safe FSM transitions: HOLD_SAFE → SOFT_RETURN (then parked)
#   - No L3/network e-stop during motion-only test
# ======================================================================

set -euo pipefail

# ------------------ CONFIG ------------------
WS="${WS:-$HOME/ros2_ws}"
LOGDIR="$WS/test_logs"
TS=$(date +"%Y%m%d_%H%M%S")
WITH_DETECTOR="${WITH_DETECTOR:-1}"   # 1=launch patient_motion_anomaly_handler, 0=don’t
MOTION_PARAMS="$WS/src/patient_engine/config/motion_thresholds_beginner.yaml"

# Scenario timings
PAUSE_SHORT=0.5
PAUSE_MED=1.0
PAUSE_LONG=2.0

mkdir -p "$LOGDIR"

# ------------------ ENV ------------------
if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo "Workspace not built. Run: colcon build --symlink-install"
  exit 1
fi

# Source with unbound-safe window (fixes COLCON_TRACE unbound var warnings)
set +u
source "$WS/install/setup.bash"
set -u

# Kill strays & clear stale FastDDS SHM
pkill -f ros2 || true
pkill -f patient_node || true
pkill -f safety_policy_manager || true
pkill -f patient_motion_anomaly_handler || true
sudo rm -f /dev/shm/fastdds_* /dev/shm/rtps_* 2>/dev/null || true

# Disable FastDDS shared memory transport to avoid SHM port lock errors
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export RMW_FASTRTPS_USE_SHM=0

# ------------------ LOG FILES ------------------
PAT_NODE_LOG="$LOGDIR/patient_node_$TS.log"
POLICY_LOG="$LOGDIR/safety_policy_$TS.log"
MOTION_LOG="$LOGDIR/patient_motion_$TS.log"

# ------------------ CLEANUP ------------------
cleanup() {
  kill ${P_PAT_NODE:-0} ${P_POLICY:-0} ${P_MOTION:-0} 2>/dev/null || true
}
trap cleanup EXIT

# ------------------ LAUNCH ------------------
echo "[1/9] Launch stack @ $TS"

echo "[launch] patient_node"
ros2 run patient_engine patient_node >"$PAT_NODE_LOG" 2>&1 & P_PAT_NODE=$!
sleep 1

echo "[launch] safety_policy_manager"
ros2 run patient_engine safety_policy_manager >"$POLICY_LOG" 2>&1 & P_POLICY=$!
sleep 1

if [[ "$WITH_DETECTOR" == "1" ]]; then
  if ros2 pkg executables patient_engine | grep -q '^patient_engine\s\+patient_motion_anomaly_handler$'; then
    if [[ -f "$MOTION_PARAMS" ]]; then
      echo "[launch] patient_motion_anomaly_handler (params: $MOTION_PARAMS)"
      ros2 run patient_engine patient_motion_anomaly_handler --ros-args \
        --params-file "$MOTION_PARAMS" >"$MOTION_LOG" 2>&1 & P_MOTION=$!
    else
      echo "[launch] patient_motion_anomaly_handler (no params file found)"
      ros2 run patient_engine patient_motion_anomaly_handler >"$MOTION_LOG" 2>&1 & P_MOTION=$!
    fi
  else
    echo "[warn] patient_motion_anomaly_handler executable not found in patient_engine"
    P_MOTION=0
  fi
else
  echo "[skip] motion detector disabled (WITH_DETECTOR=0)"
  P_MOTION=0
fi

sleep 2

# ------------------ SANITY: NODES UP ------------------
echo
echo "[2/9] Checking nodes"
for N in /patient_node /safety_policy_manager; do
  if ! ros2 node list | grep -qx "$N"; then
    echo "✗ Node $N not running"
    echo "  Check logs: $PAT_NODE_LOG $POLICY_LOG"
    exit 1
  fi
  echo "✓ Node $N is running"
done
if [[ "$WITH_DETECTOR" == "1" ]]; then
  if ros2 node list | grep -q "/patient_motion_anomaly_handler"; then
    echo "✓ Node /patient_motion_anomaly_handler is running"
  else
    echo "✗ Motion detector expected but not found"
    exit 1
  fi
fi

# ------------------ HELPERS ------------------
pub_angle()  { ros2 topic pub -1 /patient/joint_position   std_msgs/msg/Float32 "{data: $1}" >/dev/null 2>&1; }
pub_target() { ros2 topic pub -1 /patient/target_position  std_msgs/msg/Float32 "{data: $1}" >/dev/null 2>&1; }
pub_effort() { ros2 topic pub -1 /patient/effort           std_msgs/msg/Float32 "{data: $1}" >/dev/null 2>&1; }

# ------------------ SCENARIOS ------------------
echo
echo "[3/9] Scenario A: Overspeed (0° → 70° jump)"
pub_angle 0; sleep "$PAUSE_SHORT"; pub_angle 70; sleep "$PAUSE_LONG"

echo
echo "[4/9] Scenario B: Angle limit breach to 95° then back to 70°"
pub_angle 95; sleep "$PAUSE_LONG"; pub_angle 70; sleep "$PAUSE_MED"

echo
echo "[5/9] Scenario C: Drift fault (target=0°, hold 15° for 3s)"
pub_target 0; pub_angle 15; sleep 3

echo
echo "[6/9] Scenario D: Oscillation (±5° @ ~12.5 Hz for ~6s)"
for i in {1..30}; do pub_angle 5; sleep 0.1; pub_angle -5; sleep 0.1; done
sleep "$PAUSE_LONG"

echo
echo "[7/9] Scenario E: Stuck-with-force (effort high while near-still for 3s)"
pub_effort 12.0; pub_angle 45; sleep 3; pub_effort 0.0
sleep "$PAUSE_MED"

# let policy hysteresis clear back to normal
sleep 2

# ------------------ SUMMARIES ------------------
echo
echo "[8/9] Summaries (tails)"
echo "---- POLICY (MOTION_* audit re-emits) ----"
grep -E "\[policy\] MOTION_EVENT" "$POLICY_LOG" | tail -n 60 || true

echo
echo "---- POLICY (mode changes) ----"
grep -E "\[policy\] mode=" "$POLICY_LOG" | tail -n 60 || true

echo
echo "---- SAFE FSM (patient node SafeMode logs) ----"
grep -n "SafeMode" "$PAT_NODE_LOG" | tail -n 120 || true

# ------------------ VALIDATION ------------------
echo
echo "[9/9] QUICK VALIDATION"
pass=1

# 1) Did we see motion events (when detector ON)?
EVENT_COUNT=0
if [[ -f "$POLICY_LOG" ]]; then
  EVENT_COUNT=$(grep -c "\[policy\] MOTION_EVENT" "$POLICY_LOG" || true)
fi
if [[ "$WITH_DETECTOR" == "1" ]]; then
  if [[ "$EVENT_COUNT" -gt 0 ]]; then
    echo "✓ Motion events detected (via policy audit) count=$EVENT_COUNT"
  else
    echo "✗ No motion events detected (detector ON)"; pass=0
  fi
else
  echo "• Detector disabled; skipping motion-event presence check"
fi

# 2) Policy entered PASSIVE at least once
if grep -q "\[policy\] mode=passive" "$POLICY_LOG"; then
  echo "✓ Policy entered PASSIVE on anomalies"
else
  echo "✗ Policy did not enter PASSIVE"; pass=0
fi

# 3) Policy returned to NORMAL
if grep -q "\[policy\] mode=normal" "$POLICY_LOG"; then
  echo "✓ Policy returned to NORMAL after hysteresis"
else
  echo "✗ Policy did not return to NORMAL"; pass=0
fi

# 4) Patient safe FSM hit HOLD_SAFE and SOFT_RETURN
if grep -q "\[SafeMode\] -> HOLD_SAFE" "$PAT_NODE_LOG"; then
  echo "✓ Safe FSM entered HOLD_SAFE"
else
  echo "✗ Safe FSM did not enter HOLD_SAFE"; pass=0
fi

if grep -q "SOFT_RETURN complete → parked near neutral" "$PAT_NODE_LOG"; then
  echo "✓ Safe FSM completed SOFT_RETURN (parked near neutral)"
else
  echo "✗ Safe FSM did not complete SOFT_RETURN"; pass=0
fi

# 5) No L3 e-stop in motion-only test
if grep -q "NET_ESTOP_LATCH" "$POLICY_LOG"; then
  echo "✗ Unexpected L3 estop during motion-only test"; pass=0
else
  echo "✓ No ESTOP (as expected for motion-only tests)"
fi

# 6) Event spam guard (if detector on)
if [[ "$WITH_DETECTOR" == "1" ]]; then
  if [[ "$EVENT_COUNT" -gt 160 ]]; then
    echo "✗ Too many MOTION_* events ($EVENT_COUNT) → tune detector thresholds/cooldown"; pass=0
  else
    echo "• Event volume ok ($EVENT_COUNT ≤ 160)"
  fi
fi

if [[ $pass -eq 1 ]]; then
  echo "RESULT: ✅ PASS — safety policy + motion handling behaves as intended"
else
  echo "RESULT: ❌ CHECK LOGS — $LOGDIR (timestamp $TS)"
fi

echo
echo "[done] Logs:"
echo "  patient_node:          $PAT_NODE_LOG"
echo "  safety_policy_manager: $POLICY_LOG"
[[ "$WITH_DETECTOR" == "1" ]] && echo "  motion_handler:        $MOTION_LOG"
