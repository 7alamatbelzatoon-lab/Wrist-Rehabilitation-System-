#!/bin/bash
# ===============================================
# ROS 2 Wrist Rehab - Full-System Safety Test
# Runs both sides, records a rosbag, simulates MQTT loss
# ===============================================

set -euo pipefail

# --- config ---
TOPICS=(
  /patient/target_position
  /patient/joint_position
  /patient/control_mode
  /patient/freeze
  /safety/network_fault
  /safety/network_estop
  /network/warning
)
NORMAL_RUN_SECS=12     # run normally before cutting doctor MQTT
POST_FAIL_SECS=10      # observe after the cut
LOG_DIR=~/ros2_ws/test_logs
BAG_NAME=anomaly_test_$(date +%Y%m%d_%H%M%S)

mkdir -p "$LOG_DIR"
cd "$LOG_DIR"

echo "🔧 Sourcing ROS..."
# temporarily relax 'nounset' while sourcing setup files
set +u
: ${AMENT_TRACE_SETUP_FILES:=0}
source /opt/ros/jazzy/setup.bash || true
source ~/ros2_ws/install/setup.bash || true
set -u

# --- helper to clean up everything on exit ---
PIDS_TO_KILL=()
cleanup() {
  echo "🧹 Cleaning up..."
  # try graceful
  for p in "${PIDS_TO_KILL[@]}"; do
    if kill -0 "$p" 2>/dev/null; then kill "$p" 2>/dev/null || true; fi
  done
  sleep 1
  # force kill if still alive
  for p in "${PIDS_TO_KILL[@]}"; do
    if kill -0 "$p" 2>/dev/null; then kill -9 "$p" 2>/dev/null || true; fi
  done
  echo "✅ Done."
}
trap cleanup INT TERM

# --- start rosbag ---
echo "🎥 Starting rosbag: $BAG_NAME"
ros2 bag record -o "$BAG_NAME" "${TOPICS[@]}" > bag.log 2>&1 &
BAG_PID=$!
PIDS_TO_KILL+=("$BAG_PID")
sleep 1

# --- doctor side ---
echo "👨‍⚕️ Launching doctor side..."
ros2 run doctor_engine doctor_node            > doctor_node.log        2>&1 & PID_DOC_NODE=$!;   PIDS_TO_KILL+=("$PID_DOC_NODE")
ros2 run doctor_engine doctor_mqtt_bridge     > doctor_mqtt.log        2>&1 & PID_DOC_MQTT=$!;   PIDS_TO_KILL+=("$PID_DOC_MQTT")
ros2 run doctor_engine doctor_anomaly_handler > doctor_anomaly.log     2>&1 & PID_DOC_AH=$!;     PIDS_TO_KILL+=("$PID_DOC_AH")

sleep 2

# --- patient side ---
echo "🤖 Launching patient side..."
ros2 run patient_engine patient_node            > patient_node.log        2>&1 & PID_PAT_NODE=$!; PIDS_TO_KILL+=("$PID_PAT_NODE")
ros2 run patient_engine patient_mqtt_bridge     > patient_mqtt.log        2>&1 & PID_PAT_MQTT=$!; PIDS_TO_KILL+=("$PID_PAT_MQTT")
ros2 run patient_engine patient_anomaly_handler > patient_anomaly.log     2>&1 & PID_PAT_AH=$!;   PIDS_TO_KILL+=("$PID_PAT_AH")

echo "⏱️ Letting system run normally for ${NORMAL_RUN_SECS}s..."
sleep "$NORMAL_RUN_SECS"

# --- simulate network failure: cut the doctor MQTT bridge ---
echo "⚡ Simulating MQTT disconnection (killing doctor_mqtt_bridge)..."
kill "$PID_DOC_MQTT" 2>/dev/null || true

echo "👀 Observing patient safety behavior for ${POST_FAIL_SECS}s..."
sleep "$POST_FAIL_SECS"

# --- all done ---
cleanup

echo ""
echo "===================================================="
echo "✅ Test complete. Files saved in: $LOG_DIR"
echo "• Rosbag folder:  $BAG_NAME"
echo "• Logs: doctor_*.log, patient_*.log, bag.log"
echo ""
echo "Inspect bag:"
echo "  cd $LOG_DIR && ros2 bag info $BAG_NAME"
echo "Replay bag:"
echo "  cd $LOG_DIR && ros2 bag play $BAG_NAME"
echo "Plot:"
echo '  rqt_plot /patient/joint_position /patient/target_position'
echo "===================================================="

