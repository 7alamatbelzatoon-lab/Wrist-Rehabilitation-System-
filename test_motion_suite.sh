#!/usr/bin/env bash
# test_motion_suite.sh — Patient-side motion anomaly end-to-end test
# Brings up: patient_node, patient_motion_anomaly_handler (with params), safety_policy_manager
# Drives scenarios A–E and validates that policy & safe FSM react as intended.

set -Eeuo pipefail

# ---------------- env & prep ----------------
ROS_WS="${ROS_WS:-$HOME/ros2_ws}"
PKG="patient_engine"
EXE_NODE="patient_node"
EXE_MOTION="patient_motion_anomaly_handler"
EXE_POLICY="safety_policy_manager"
PARAMS_FILE="$ROS_WS/src/$PKG/config/motion_thresholds_beginner.yaml"

TS="$(date +"%Y%m%d_%H%M%S")"
LOGDIR="$ROS_WS/test_logs"
mkdir -p "$LOGDIR"

L_NODE="$LOGDIR/patient_node_${TS}.log"
L_MOTION="$LOGDIR/patient_motion_${TS}.log"
L_POLICY="$LOGDIR/safety_policy_${TS}.log"
SUMMARY="$LOGDIR/summary_${TS}.txt"

say(){ echo -e "$*"; }
die(){ echo "ERROR: $*" >&2; exit 1; }

# Make DDS use UDP (avoid SHM port lockups seen earlier)
export RMW_FASTRTPS_USE_SHM=0

# Source ROS 2 & workspace safely (avoid COLCON_TRACE unbound)
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
  set +u; source /opt/ros/jazzy/setup.bash; set -u
fi
if [ -f "$ROS_WS/install/setup.bash" ]; then
  set +u; source "$ROS_WS/install/setup.bash"; set -u
else
  die "Workspace not built. Run: colcon build --symlink-install"
fi

say "[env] workspace: $ROS_WS"

# Aggressive cleanup
say "[clean] killing strays & clearing DDS shm..."
pkill -f ros2 || true
pkill -f "$EXE_NODE" || true
pkill -f "$EXE_MOTION" || true
pkill -f "$EXE_POLICY" || true
# Clear FastDDS SHM (no sudo needed when SHM is user-owned; ignore errors)
rm -f /dev/shm/fastdds_* /dev/shm/rtps_* 2>/dev/null || true

# Ensure params file exists
[ -f "$PARAMS_FILE" ] || die "Params file not found: $PARAMS_FILE"

# ---------------- launch ----------------
say "[launch] starting nodes..."
(ros2 run "$PKG" "$EXE_NODE"   >"$L_NODE"   2>&1) & P_NODE=$!
(ros2 run "$PKG" "$EXE_MOTION" --ros-args --params-file "$PARAMS_FILE" >"$L_MOTION" 2>&1) & P_MOTION=$!
(ros2 run "$PKG" "$EXE_POLICY" >"$L_POLICY" 2>&1) & P_POLICY=$!

cleanup(){
  kill $P_NODE 2>/dev/null || true
  kill $P_MOTION 2>/dev/null || true
  kill $P_POLICY 2>/dev/null || true
}
trap cleanup EXIT

# ---------------- readiness checks ----------------
have_node(){ ros2 node list 2>/dev/null | grep -q "$1"; }

say "[check] waiting for nodes to appear..."
for i in {1..60}; do
  A=$(have_node "/$EXE_NODE" && echo ok || echo no)
  B=$(have_node "/$EXE_MOTION" && echo ok || echo no)
  C=$(have_node "/$EXE_POLICY" && echo ok || echo no)
  if [ "$A$B$C" = "okokok" ]; then break; fi
  sleep 0.3
done
if ! have_node "/$EXE_NODE" || ! have_node "/$EXE_MOTION" || ! have_node "/$EXE_POLICY"; then
  say "⚠ node list:"
  ros2 node list || true
  die "nodes failed to come up"
fi
say "✓ nodes are up"

# Sanity: the motion handler should advertise /safety/motion_event
# (don’t fail if not visible immediately)
sleep 0.5

# ---------------- helpers ----------------
pub_angle () { ros2 topic pub -1 /patient/joint_position std_msgs/msg/Float32 "{data: $1}" >/dev/null 2>&1; }
pub_target() { ros2 topic pub -1 /patient/target_position std_msgs/msg/Float32 "{data: $1}" >/dev/null 2>&1; }
pub_effort() { ros2 topic pub -1 /patient/effort std_msgs/msg/Float32 "{data: $1}" >/dev/null 2>&1; }

# ---------------- scenarios ----------------
say
say "[scenarios] driving motion anomalies"

say "[A] Overspeed (0° → 70° jump)"
pub_angle 0; sleep 0.05; pub_angle 70; sleep 2.0

say "[B] Angle limit breach (95° → 70°)"
pub_angle 95; sleep 1.0; pub_angle 70; sleep 1.5

say "[C] Drift (target 0°, position 15° held)"
pub_target 0; pub_angle 15; sleep 3.0

say "[D] Oscillation (±5° @ ~10 Hz)"
for i in {1..20}; do pub_angle 5; sleep 0.1; pub_angle -5; sleep 0.1; done
sleep 1.0

say "[E] Stuck-with-force (effort=12 near stillness)"
pub_effort 12.0; pub_angle 45; sleep 3.0; pub_effort 0.0
sleep 1.0

# Give policy hysteresis time to clear back to normal
sleep 2.5

# ---------------- collect & summarize ----------------
say
say "[collect] tails & summary → $SUMMARY"

{
  echo "=== MOTION HANDLER (grep MOTION_) ==="
  grep -E "MOTION_|ACCEL|OVERSPEED|STALE|DRIFT|OSCILLATION" "$L_MOTION" | tail -n 80 || true
  echo
  echo "=== POLICY (MOTION_* audit re-emits) ==="
  grep -E "\[policy\] MOTION_EVENT" "$L_POLICY" | tail -n 80 || true
  echo
  echo "=== POLICY (mode changes) ==="
  grep -E "\[policy\] mode=" "$L_POLICY" | tail -n 120 || true
  echo
  echo "=== SAFE FSM (patient_node SafeMode logs) ==="
  nl -ba "$L_NODE" | grep -E "SafeMode|HOLD_SAFE|SOFT_RETURN|parked near neutral" | tail -n 200 || true
} > "$SUMMARY"

# ---------------- validation ----------------
say "[validate] quick checks"

have_motion_events=false
grep -q "MOTION_EVENT" "$L_POLICY" && have_motion_events=true

entered_passive=false
grep -q "mode=passive" "$L_POLICY" && entered_passive=true

returned_normal=false
grep -q "mode=normal" "$L_POLICY" && returned_normal=true

entered_hold=false
grep -q "HOLD_SAFE" "$L_NODE" && entered_hold=true

did_soft_return=false
grep -q "SOFT_RETURN complete → parked near neutral" "$L_NODE" && did_soft_return=true

# Count policy MOTION events to detect spam
event_count=$(grep -c "MOTION_EVENT" "$L_POLICY" || true)

$have_motion_events   && say "✓ motion events detected (via policy audit)" || say "✗ no motion events detected"
$entered_passive      && say "✓ policy entered PASSIVE on anomalies"       || say "✗ policy did not enter PASSIVE"
$returned_normal      && say "✓ policy returned to NORMAL after hysteresis" || say "✗ policy did not return to NORMAL"
$entered_hold         && say "✓ safe FSM entered HOLD_SAFE"                 || say "✗ safe FSM did not enter HOLD_SAFE"
$did_soft_return      && say "✓ safe FSM completed SOFT_RETURN (parked near neutral)" || say "✗ safe FSM did not complete SOFT_RETURN"

if [ "$event_count" -gt 120 ]; then
  say "✗ too many MOTION_* events ($event_count) → likely spam; review detector cooldown/thresholds"
else
  say "✓ event volume reasonable (count=$event_count)"
fi

if $have_motion_events && $entered_passive && $returned_normal && $entered_hold && $did_soft_return; then
  say; say "RESULT: ✅ PASS — motion anomaly pipeline healthy"
else
  say; say "RESULT: ❌ CHECK LOGS (see $LOGDIR, ts $TS)"
fi

say "[done] logs:"
say "  node:   $L_NODE"
say "  motion: $L_MOTION"
say "  policy: $L_POLICY"
say "  summary:$SUMMARY"
