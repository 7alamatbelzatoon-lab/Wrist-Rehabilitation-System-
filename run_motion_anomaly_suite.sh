#!/bin/bash
# ======================================================================
# run_motion_anomaly_suite.sh  —  Full-stack motion anomaly validation
# Stack (5 nodes):
#   patient_engine: patient_node, patient_motion_anomaly_handler, patient_network_anomaly_handler
#   doctor_engine : doctor_node (optional, passive listener)
#
# Scenarios:
#  A) Overspeed detection
#  B) Angle limit breach
#  C) Drift fault
#  D) Oscillation / instability
#  E) Stuck-with-force (if effort topic available)
#
# PASS criteria:
#  - L2 or L3 motion faults detected
#  - Proper FSM responses in patient_node log
#  - Faults clear correctly after hysteresis period
# ======================================================================

set -u

WS="$HOME/ros2_ws"
LOGDIR="$WS/test_logs"
TS=$(date +"%Y%m%d_%H%M%S")
mkdir -p "$LOGDIR"

# --- Env / workspace ---
if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"
else
  echo "Workspace not built. Run: colcon build --symlink-install"
  exit 1
fi

echo
echo "[1/8] Launching motion anomaly stack @ $TS"

PAT_NODE="$LOGDIR/patient_node_$TS.log"
PAT_MOTION="$LOGDIR/patient_motion_$TS.log"
PAT_NET="$LOGDIR/patient_network_$TS.log"

# --- Start nodes ---
ros2 run patient_engine patient_node                   >"$PAT_NODE"    2>&1 & P_PAT_NODE=$!
ros2 run patient_engine patient_motion_anomaly_handler  --ros-args --params-file src/patient_engine/config/motion_thresholds_beginner.yaml >"$PAT_MOTION" 2>&1 & P_PAT_MOTION=$!
ros2 run patient_engine patient_network_anomaly_handler >"$PAT_NET"     2>&1 & P_PAT_NET=$!

sleep 5

# Helper: publish once
pub_angle() {
  ros2 topic pub -1 /patient/joint_position std_msgs/msg/Float32 "{data: $1}" >/dev/null 2>&1
}

# ======================================================================
# Scenario A: Overspeed detection
# ======================================================================
echo
echo "[2/8] Scenario A: Overspeed"
echo "[simulate] Rapid position jump from 0 → 70°"
pub_angle 0
sleep 0.05
pub_angle 70
sleep 3

# ======================================================================
# Scenario B: Angle limit breach
# ======================================================================
echo
echo "[3/8] Scenario B: Angle limit breach"
echo "[simulate] Exceeding joint limit to 95°"
pub_angle 95
sleep 3

# ======================================================================
# Scenario C: Drift fault
# ======================================================================
echo
echo "[4/8] Scenario C: Drift fault"
echo "[simulate] Hold steady 15° away from target for 3s"
ros2 topic pub -1 /patient/target_position std_msgs/msg/Float32 "{data: 0.0}" >/dev/null 2>&1
pub_angle 15
sleep 3

# ======================================================================
# Scenario D: Oscillation
# ======================================================================
echo
echo "[5/8] Scenario D: Oscillation (rapid alternating motion)"
for i in {1..20}; do
  pub_angle 5
  sleep 0.1
  pub_angle -5
  sleep 0.1
done
sleep 3

# ======================================================================
# Scenario E: Stuck with force (if /effort topic active)
# ======================================================================
echo
echo "[6/8] Scenario E: Stuck-with-force (simulated high effort)"
ros2 topic pub -1 /patient/effort std_msgs/msg/Float32 "{data: 12.0}" >/dev/null 2>&1
pub_angle 45
sleep 3
ros2 topic pub -1 /patient/effort std_msgs/msg/Float32 "{data: 0.0}" >/dev/null 2>&1

# ======================================================================
# Summaries
# ======================================================================
echo
echo "[7/8] Summaries"
echo "---- MOTION LOG ----"
grep -E "FAULT|ESTOP|CLEAR" "$PAT_MOTION" | tail -n 30
echo
echo "---- NETWORK HANDLER LOG ----"
grep -E "mode=|CLEAR" "$PAT_NET" | tail -n 20
echo
echo "---- SAFE FSM TIMELINE ----"
grep -n "SafeMode" "$PAT_NODE" | tail -n 200

# ======================================================================
# Validation
# ======================================================================
echo
echo "[8/8] QUICK VALIDATION"
pass=1

if grep -q "L2 FAULT" "$PAT_MOTION"; then
  echo "✓ Motion fault(s) detected"
else
  echo "✗ No motion fault detected"; pass=0
fi

if grep -q "L3 ESTOP" "$PAT_MOTION"; then
  echo "✓ Motion ESTOP detected"
else
  echo "• No ESTOP observed (ok if thresholds not exceeded)"
fi

if grep -q "CLEAR → normal" "$PAT_MOTION"; then
  echo "✓ Motion clear events observed"
else
  echo "✗ No clear-to-normal log"; pass=0
fi

if [ $pass -eq 1 ]; then
  echo "RESULT: ✅ PASS — motion anomaly suite stable"
else
  echo "RESULT: ❌ CHECK LOGS (see $LOGDIR, timestamp $TS)"
fi

# Cleanup
kill ${P_PAT_NODE:-0} ${P_PAT_MOTION:-0} ${P_PAT_NET:-0} 2>/dev/null || true
echo "[done] Logs: $LOGDIR (timestamp $TS)"
