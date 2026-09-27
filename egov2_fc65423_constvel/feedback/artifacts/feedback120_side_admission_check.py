"""Offline source/log check for Feedback119's rejected static-safe A* class.

Feedback119 did not retain raw path coordinates, so this verifies the saved
decision and current admission chain rather than claiming exact path replay.
"""
from pathlib import Path
import re

root = Path(__file__).resolve().parents[2]
source = (root / "ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/"
          "planner/plan_manage/src/planner_manager.cpp").read_text()
log = (root / "runs/20260926_031106_562007/roslaunch_stdout.log").read_text(
    errors="replace")

old = re.search(r"\[side-semantic-v2\][^\n]*path_side_valid=0[^\n]*", log)
assert old, "Feedback119 rejected-side record missing"
assert "raw_path_static_free=1" in log and "verdict=GUIDE_REJECTED" in log
print("Feedback119 rejected route: " + old.group(0)[:320])

raw = source[source.index("const auto raw_path_usable"):
             source.index("std::vector<Eigen::Vector3d> raw_path", source.index("const auto raw_path_usable"))]
assert "getInflateOccupancy" in raw and "return true;" in raw
assert "side_semantic_valid" not in raw and "side_intent" not in raw
assert "if (path_static_free && raw_path.size() >= 2)" in source
assert "if (repaired_side_valid && path_static_free && grid_map_" in source
assert "astar_base_valid && path_static_free &&\n                                          repaired_side_valid" in source
assert "GUIDE_REJECTED" not in source
assert "SIDE_SEMANTIC_INVALID" not in source
assert "used_as_hard_gate=0" in source
assert "return side_semantic_valid" not in source
assert not re.search(r"&&\s*path_side_valid", source)
print("Current raw static-safe route -> guide -> corridor: PASS; path-wide SIDE hard gates: 0")
print("LOS hard reject remains outside the changed source region; source diff checked separately")
