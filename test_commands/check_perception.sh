#!/usr/bin/env bash
# One command live test of the perception node (Requirement 1).
# Starts Gazebo (no RViz) + aruco_detector_node, runs check_perception.py, then shuts EVERYTHING down.
#   bash ~/ros2_ws/src/erl1_assignment1/test_commands/check_perception.sh
# Logs and the saved /marker_image go to /tmp/perception_check/

set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT=/tmp/perception_check
mkdir -p "$OUT"

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
PARAMS="$(ros2 pkg prefix erl1_assignment1)/share/erl1_assignment1/config/assignment_sim.yaml"
[ -f "$PARAMS" ] || { echo "params file not found: $PARAMS (did you build?)"; exit 1; }

cleanup() {
  echo; echo "== shutting down"
  [ -n "${DET_PID:-}" ] && kill -INT -- -"$DET_PID" 2>/dev/null
  [ -n "${SIM_PID:-}" ] && kill -INT -- -"$SIM_PID" 2>/dev/null
  sleep 3
  pkill -f aruco_detector_node 2>/dev/null
  pkill -f "gz sim" 2>/dev/null; pkill -f parameter_bridge 2>/dev/null; pkill -f image_bridge 2>/dev/null
  pkill -f robot_state_publisher 2>/dev/null; pkill -f ekf_node 2>/dev/null; pkill -f rviz2 2>/dev/null
  sleep 1
  pkill -9 -f "gz sim" 2>/dev/null
  echo "done. logs: $OUT/sim.log  $OUT/detector.log"
}
trap cleanup EXIT INT TERM

# leftovers from a previous run would publish the same topics
pkill -f "gz sim" 2>/dev/null; pkill -f aruco_detector_node 2>/dev/null; sleep 1

echo "== starting simulation (log: $OUT/sim.log)"
setsid ros2 launch erl1_assignment1 spawn_assignment.launch.py rviz:=false > "$OUT/sim.log" 2>&1 &
SIM_PID=$!

echo "== waiting for /camera/image"
for i in $(seq 1 60); do
  if timeout 3 ros2 topic echo --once /camera/image --field header.frame_id >/dev/null 2>&1; then
    echo "   camera is up after ~$((i*3)) s"; break
  fi
  if ! kill -0 "$SIM_PID" 2>/dev/null; then echo "simulation died, see $OUT/sim.log"; tail -20 "$OUT/sim.log"; exit 1; fi
  [ "$i" = 60 ] && { echo "no camera image after 3 min, see $OUT/sim.log"; exit 1; }
done

echo "== starting aruco_detector_node (log: $OUT/detector.log)"
setsid ros2 run erl1_assignment1 aruco_detector_node.py --ros-args --params-file "$PARAMS" \
  > "$OUT/detector.log" 2>&1 &
DET_PID=$!
sleep 3
if ! kill -0 "$DET_PID" 2>/dev/null; then echo "detector died:"; cat "$OUT/detector.log"; exit 1; fi

echo "== parameters"
for p in image_topic camera_info_topic detection_topic marker_image_topic aruco_dictionary_name marker_size_meters use_sim_time; do
  printf '   %-24s %s\n' "$p" "$(ros2 param get /aruco_detector_node $p 2>&1 | sed 's/.*value is: //')"
done

echo "== checks"
python3 "$HERE/check_perception.py" "$OUT"
RC=$?

echo "== detector log"
cat "$OUT/detector.log"
exit $RC
