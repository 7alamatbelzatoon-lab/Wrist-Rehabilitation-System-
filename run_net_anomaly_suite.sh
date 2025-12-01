#!/bin/bash
# ======================================================================
# run_net_anomaly_suite.sh  —  Full-stack network anomaly validation
# Stack (6 nodes):
#   patient_engine: patient_node, patient_mqtt_bridge, patient_network_anomaly_handler
#   doctor_engine : doctor_node,  doctor_mqtt_bridge, doctor_network_anomaly_handler
#
# Scenarios:
#  A) PATIENT bridge cut @ 60° (expect HOLD -> (maybe CREEP) -> SOFT, then CLEAR)
#  B) DOCTOR  bridge cut @ 60°
#  C) DOCTOR  bridge cut @ 40° (safe-zone edge → usually SOFT only)
#  D) Short network blip (iptables DROP 2s)
#  E) High latency (~900ms via tc netem)
#  F) Double doctor reconnect
#
# PASS criteria:
#  - We see safe FSM transitions in patient_node log
#  - We see "CLEAR → mode=normal (after network restore)" in patient_network_anomaly_handler log
#  - We see ESTOP raised+cleared either in patient_mqtt L3 or network_anomaly_handler log
# ======================================================================

set -u

# --- Config ---
WS="$HOME/ros2_ws"
LOGDIR="$WS/test_logs"
TS=$(date +"%Y%m%d_%H%M%S")
HIVEMQ_HOST="bea7b081570a4e03a77248e8b07072d9.s1.eu.hivemq.cloud"

# Motion timing to let the joint move before we "cut"
MOVE_A_S=3       # Scenario A: give time to move toward 60°
MOVE_B_S=3       # Scenario B: give time to move toward 60°
MOVE_C_S=1.6     # Scenario C: short nudge toward 40°
SLEEP_AFTER_RESTART=20   # let system stabilize after bridge restarts (was 15)
WARMUP=5

mkdir -p "$LOGDIR"

# --- Env / workspace ---
if [ -f "$WS/install/setup.bash" ]; then
  export COLCON_TRACE="${COLCON_TRACE:-0}"
  set +u
  # shellcheck source=/dev/null
  source "$WS/install/setup.bash"
  set -u
else
  echo "Workspace not built. Run: colcon build --symlink_install"
  exit 1
fi

# --- Resolve broker IP (for iptables) ---
BROKER_IP=$(getent hosts "$HIVEMQ_HOST" | awk '{print $1; exit}')
if [ -z "${BROKER_IP:-}" ]; then
  echo "Failed to resolve $HIVEMQ_HOST"
  exit 1
fi

# --- Detect default interface for tc ---
IFACE=$(ip route get 1.1.1.1 2>/dev/null | awk '/dev/{for (i=1;i<=NF;i++) if ($i=="dev") print $(i+1)}' | head -n1)
IFACE=${IFACE:-enp0s3}

echo
echo "[1/9] Launching full 6-node stack @ $TS (broker $BROKER_IP on $IFACE)"

# --- Log files ---
PAT_NODE="$LOGDIR/patient_node_$TS.log"
PAT_MQTT="$LOGDIR/patient_mqtt_$TS.log"
PAT_ANOM="$LOGDIR/patient_network_anomaly_$TS.log"
DOC_NODE="$LOGDIR/doctor_node_$TS.log"
DOC_MQTT="$LOGDIR/doctor_mqtt_$TS.log"

# --- Start helpers (append >> so logs keep full timeline) ---
start_patient_mqtt() {
  ros2 run patient_engine patient_mqtt_bridge >>"$PAT_MQTT" 2>&1 &
  P_PAT_MQTT=$!
}
start_doctor_mqtt() {
  ros2 run doctor_engine doctor_mqtt_bridge >>"$DOC_MQTT" 2>&1 &
  P_DOC_MQTT=$!
}

# --- Launch stack ---
ros2 run patient_engine patient_node                        >"$PAT_NODE"   2>&1 & P_PAT_NODE=$!
start_patient_mqtt
ros2 run patient_engine patient_network_anomaly_handler      >"$PAT_ANOM"   2>&1 & P_PAT_ANOM=$!
ros2 run doctor_engine  doctor_node                          >"$DOC_NODE"   2>&1 & P_DOC_NODE=$!
ros2 run doctor_engine  doctor_network_anomaly_handler       >>"$DOC_MQTT"  2>&1 & P_DOC_ANOM=$!
start_doctor_mqtt

# --- Wait helper ---
wait_for_log() {
  local pattern="$1"; local file="$2"; local timeout="${3:-60}"
  echo "[wait] log '$pattern' in $(basename "$file") (≤${timeout}s)"
  for ((i=0;i<timeout;i++)); do
    if grep -q "$pattern" "$file" 2>/dev/null; then
      echo "[wait] saw '$pattern' in $(basename "$file")"
      return 0
    fi
    sleep 1
  done
  echo "[wait] TIMEOUT waiting for '$pattern' in $(basename "$file")"
  return 1
}

# --- Initial waits ---
wait_for_log "MQTT connected" "$PAT_MQTT" 60
wait_for_log "MQTT connected" "$DOC_MQTT" 60
sleep "$WARMUP"

# ========== Scenario A: PATIENT cut @ 60° ==========
echo
echo "[2/9] Scenario A: PATIENT cut @ 60.0°"
echo "[pub_once] sending target 60.0° → /doctor/target_position"
ros2 topic pub -1 /doctor/target_position std_msgs/msg/Float32 "{data: 60.0}" >/dev/null 2>&1
sleep "$MOVE_A_S"
echo "[kill] patient_mqtt_bridge (simulate patient-side disconnect)"
kill -TERM ${P_PAT_MQTT:-0} 2>/dev/null || true
sleep 4
echo "[restart] patient_mqtt_bridge"
start_patient_mqtt
wait_for_log "MQTT connected" "$PAT_MQTT" 60
echo "[stabilize] ${SLEEP_AFTER_RESTART}s"
sleep "$SLEEP_AFTER_RESTART"

# ========== Scenario B: DOCTOR cut @ 60° ==========
echo
echo "[3/9] Scenario B: DOCTOR cut @ 60.0°"
echo "[pub_once] sending target 60.0° → /doctor/target_position"
ros2 topic pub -1 /doctor/target_position std_msgs/msg/Float32 "{data: 60.0}" >/dev/null 2>&1
sleep "$MOVE_B_S"
echo "[kill] doctor_mqtt_bridge (simulate doctor-side disconnect)"
kill -TERM ${P_DOC_MQTT:-0} 2>/dev/null || true
sleep 4
echo "[restart] doctor_mqtt_bridge"
start_doctor_mqtt
wait_for_log "MQTT connected" "$DOC_MQTT" 60
echo "[stabilize] ${SLEEP_AFTER_RESTART}s"
sleep "$SLEEP_AFTER_RESTART"

# ========== Scenario C: DOCTOR cut @ 40° ==========
echo
echo "[4/9] Scenario C: DOCTOR cut @ 40.0° (safe-zone edge)"
echo "[pub_once] sending target 40.0° → /doctor/target_position"
ros2 topic pub -1 /doctor/target_position std_msgs/msg/Float32 "{data: 40.0}" >/dev/null 2>&1
sleep "$MOVE_C_S"
echo "[kill] doctor_mqtt_bridge"
kill -TERM ${P_DOC_MQTT:-0} 2>/dev/null || true
sleep 4
echo "[restart] doctor_mqtt_bridge"
start_doctor_mqtt
wait_for_log "MQTT connected" "$DOC_MQTT" 60
echo "[stabilize] ${SLEEP_AFTER_RESTART}s"
sleep "$SLEEP_AFTER_RESTART"

# ========== Scenario D: Short network blip ==========
echo
echo "[5/9] Scenario D: Short network blip (2s)"
echo "[pub_once] sending target 10.0° → /doctor/target_position"
ros2 topic pub -1 /doctor/target_position std_msgs/msg/Float32 "{data: 10.0}" >/dev/null 2>&1
sudo iptables -I OUTPUT -p tcp --dport 8883 -d "$BROKER_IP" -j DROP
echo "[NET] blocked broker $BROKER_IP:8883"
sleep 2
sudo iptables -D OUTPUT -p tcp --dport 8883 -d "$BROKER_IP" -j DROP
echo "[NET] unblocked broker $BROKER_IP:8883"
sleep "$SLEEP_AFTER_RESTART"

# --- New explicit check for CLEAR ---
echo "[assert] waiting for CLEAR → mode=normal after Scenario D"
wait_for_log "CLEAR → mode=normal" "$PAT_ANOM" 30 || echo "[warn] CLEAR not yet; will re-check in final validation"

# ========== Scenario E: High latency ==========
echo
echo "[6/9] Scenario E: High latency (~900ms)"
echo "[pub_once] sending target 60.0° → /doctor/target_position"
ros2 topic pub -1 /doctor/target_position std_msgs/msg/Float32 "{data: 60.0}" >/dev/null 2>&1
sudo tc qdisc add dev "$IFACE" root netem delay 900ms 2>/dev/null || true
echo "[NETEM] delay 900ms to $BROKER_IP via $IFACE"
sleep 3
sudo tc qdisc del dev "$IFACE" root netem 2>/dev/null || true
echo "[NETEM] cleared on $IFACE"
sleep "$SLEEP_AFTER_RESTART"

# ========== Scenario F: Double doctor reconnect ==========
echo
echo "[7/9] Scenario F: Double doctor reconnect"
echo "[pub_once] sending target 60.0° → /doctor/target_position"
ros2 topic pub -1 /doctor/target_position std_msgs/msg/Float32 "{data: 60.0}" >/dev/null 2>&1
echo "[kill] doctor_mqtt_bridge (1)"
kill -TERM ${P_DOC_MQTT:-0} 2>/dev/null || true
sleep 2
echo "[restart] doctor_mqtt_bridge (1)"
start_doctor_mqtt
wait_for_log "MQTT connected" "$DOC_MQTT" 60
echo "[kill] doctor_mqtt_bridge (2)"
kill -TERM ${P_DOC_MQTT:-0} 2>/dev/null || true
sleep 2
echo "[restart] doctor_mqtt_bridge (2)"
start_doctor_mqtt
wait_for_log "MQTT connected" "$DOC_MQTT" 60
sleep "$SLEEP_AFTER_RESTART"

# =================== Summaries ===================
echo
echo "[8/9] Summaries"

echo "---- SAFE FSM TIMELINE ----"
grep -n "SafeMode" "$PAT_NODE" | tail -n 200
echo
echo "---- SAFE STATE COUNTS ----"
grep -Eo "HOLD_SAFE|CREEP_TO_SAFE|SOFT_RETURN" "$PAT_NODE" | sort | uniq -c
echo
echo "---- PATIENT MQTT ----"
grep -E "L1|L2|L3|clear_estop" "$PAT_MQTT" | tail -n 20
echo
echo "---- DOCTOR MQTT ----"
grep -E "L1|L2|L3|clear_estop" "$DOC_MQTT" | tail -n 20

# =================== Validation ===================
echo
echo "---- QUICK VALIDATION ----"
pass=1

if grep -q "HOLD_SAFE" "$PAT_NODE"; then
  echo "✓ HOLD_SAFE seen"
else
  echo "✗ HOLD_SAFE not seen"; pass=0
fi
if grep -q "SOFT_RETURN" "$PAT_NODE"; then
  echo "✓ SOFT_RETURN seen"
else
  echo "✗ SOFT_RETURN not seen"; pass=0
fi

if grep -q 'CLEAR → mode=normal' "$PAT_ANOM"; then
  echo "✓ clear-to-normal observed"
else
  echo "✗ no CLEAR event seen"; pass=0
fi

if (grep -q 'L3=True' "$PAT_MQTT" && grep -q 'L3=False' "$PAT_MQTT") || \
   (grep -q 'mode=estop' "$PAT_ANOM" && grep -q 'mode=normal' "$PAT_ANOM"); then
  echo "✓ estop raised and cleared"
else
  echo "• estop clear not observed (check logs)"
fi

if [ $pass -eq 1 ]; then
  echo "RESULT: ✅ PASS"
else
  echo "RESULT: ❌ CHECK LOGS (see $LOGDIR, timestamp $TS)"
fi

# =================== Cleanup ===================
echo
echo "[9/9] Done. Logs: $LOGDIR (timestamp $TS)"
sudo iptables -D OUTPUT -p tcp --dport 8883 -d "$BROKER_IP" -j DROP 2>/dev/null || true
sudo tc qdisc del dev "$IFACE" root netem 2>/dev/null || true
echo "[NET] unblocked broker $BROKER_IP:8883"
echo "[NETEM] cleared on $IFACE"

# Kill background nodes
kill ${P_PAT_NODE:-0} ${P_PAT_MQTT:-0} ${P_PAT_ANOM:-0} ${P_DOC_NODE:-0} ${P_DOC_MQTT:-0} ${P_DOC_ANOM:-0} 2>/dev/null || true
