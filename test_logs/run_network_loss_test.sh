#!/usr/bin/env bash
set -euo pipefail

# --- setup ---
WS=~/ros2_ws
LOGDIR="$WS/test_logs"
mkdir -p "$LOGDIR"
TS=$(date +%Y%m%d_%H%M%S)

BROKER_HOST="bea7b081570a4e03a77248e8b07072d9.s1.eu.hivemq.cloud"
BROKER_IP=$(getent hosts "$BROKER_HOST" | awk '{print $1}' | head -n1)
BROKER_PORT=8883

echo "Broker resolved to $BROKER_IP"
echo "Starting full system test @ $TS"
echo

# helper funcs
block_net() {
  sudo iptables -I OUTPUT -p tcp --dport $BROKER_PORT -d $BROKER_IP -j DROP
  sudo iptables -I INPUT  -p tcp --sport $BROKER_PORT -s $BROKER_IP -j DROP
  echo "[NET] blocked → connection lost"
}
unblock_net() {
  sudo iptables -D OUTPUT -p tcp --dport $BROKER_PORT -d $BROKER_IP -j DROP || true
  sudo iptables -D INPUT  -p tcp --sport $BROKER_PORT -s $BROKER_IP -j DROP || true
  echo "[NET] unblocked → connection restored"
}
cleanup() { unblock_net; }
trap cleanup EXIT

# --- launch nodes ---

echo "[1/6] Launching patient side..."
ros2 run rehab patient_mqtt_bridge  > "$LOGDIR/patient_mqtt_$TS.log" 2>&1 &
sleep 2
ros2 run rehab patient_node         > "$LOGDIR/patient_node_$TS.log" 2>&1 &
sleep 3

echo "[2/6] Launching doctor side..."
ros2 run rehab doctor_mqtt_bridge   > "$LOGDIR/doctor_mqtt_$TS.log" 2>&1 &
sleep 3
ros2 run rehab doctor_node          > "$LOGDIR/doctor_node_$TS.log" 2>&1 &
sleep 3

echo "[3/6] Warm-up 5s for sync..."
sleep 5

# --- start scenarios ---
echo
echo "=== Scenario A: Far-angle dropout (expect HOLD→CREEP→SOFT) ==="
echo "[Auto] Sending 55° target..."
ros2 topic pub /doctor/target_position std_msgs/Float32 "{data: 55.0}" -1
sleep 2
block_net
sleep 6
unblock_net
sleep 5

echo
echo "=== Scenario B: Near-neutral dropout (expect HOLD→SOFT) ==="
echo "[Auto] Sending 5° target..."
ros2 topic pub /doctor/target_position std_msgs/Float32 "{data: 5.0}" -1
sleep 2
block_net
sleep 4
unblock_net
sleep 4

echo
echo "=== Scenario C: Short blip (expect no flapping) ==="
ros2 topic pub /doctor/target_position std_msgs/Float32 "{data: 25.0}" -1
sleep 3
block_net
sleep 2
unblock_net
sleep 4

echo
echo "[4/6] Reset to neutral..."
ros2 topic pub /doctor/target_position std_msgs/Float32 "{data: 0.0}" -1
sleep 3

# --- Summaries ---
echo
echo "[5/6] Collecting summaries..."
TARGET_LOG="$LOGDIR/patient_node_$TS.log"

echo
echo "---- SAFE FSM TIMELINE ----"
grep -nE 'HOLD_SAFE|CREEP_TO_SAFE|SOFT_RETURN' "$TARGET_LOG" || true

echo
echo "---- STATE COUNTS ----"
grep -E 'HOLD_SAFE|CREEP_TO_SAFE|SOFT_RETURN' "$TARGET_LOG" \
 | sed -E 's/.*(HOLD_SAFE|CREEP_TO_SAFE|SOFT_RETURN).*/\1/' \
 | sort | uniq -c || true

echo
echo "---- NETWORK STATES (Doctor Bridge) ----"
grep -nE 'L1=|L2=|L3=' "$LOGDIR/doctor_mqtt_$TS.log" | tail -n 20 || true

echo
echo "[6/6] Logs stored in $LOGDIR"
echo "Test timestamp: $TS"
echo "Done ✅"
