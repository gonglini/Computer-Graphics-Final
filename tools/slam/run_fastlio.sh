#!/bin/bash
# Run FAST-LIO on a bag made by make_bag.py and collect the results next to it.
#   wsl -d Ubuntu-20.04 -u root -- bash /mnt/c/.../tools/slam/run_fastlio.sh /mnt/c/.../sim_mapping.bag
set -e
BAG="$1"
OUT="$(dirname "$BAG")/fastlio_result"
WS=/opt/fastlio_ws
mkdir -p "$OUT"
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
mkdir -p "$WS/src/FAST_LIO/PCD"
rm -f "$WS/src/FAST_LIO/PCD/"*.pcd
cp "$(dirname "$0")/sim.yaml" "$WS/src/FAST_LIO/config/sim.yaml"

roscore > "$OUT/roscore.log" 2>&1 &
RC=$!
sleep 4
rosparam set use_sim_time true
roslaunch fast_lio mapping_sim.launch > "$OUT/fastlio.log" 2>&1 &
FL=$!
sleep 4
# estimated trajectory: FAST-LIO's /Odometry, written as CSV while the bag plays
rostopic echo -p /Odometry > "$OUT/odometry.csv" &
EC=$!
sleep 1
rosbag play --clock -r 1.0 "$BAG"
sleep 20
kill -INT $EC || true
kill -INT $FL || true      # FAST-LIO saves PCD/scans.pcd on shutdown
wait $FL || true
kill -INT $RC || true
sleep 2
mkdir -p "$OUT/pcd"
rm -f "$OUT/pcd/"*.pcd
cp "$WS/src/FAST_LIO/PCD/"*.pcd "$OUT/pcd/" || true
echo "results in $OUT"
ls -la "$OUT"
