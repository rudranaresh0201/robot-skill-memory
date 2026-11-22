import math

result = {"localized": False, "moved_above": False, "pressed": False, "done": False}

def run():
    loc = localize_object("flat stove")
    if not loc.ok:
        return
    result["localized"] = True
    c = loc.geometry.pose.position
    x, y, z = c[0], c[1], c[2]
    state = get_robot_state()
    ee = state.end_effector_poses if hasattr(state, "end_effector_poses") else None
    quat = [0.0, 1.0, 0.0, 0.0]
    frame = loc.geometry.pose.frame
    close_gripper()
    # The knob is usually at the front (toward the robot) edge of the stove
    above = Pose([x, y, z + 0.20], quat, frame)
    r = move_to_pose(above, 0.02, 200)
    if not r.ok:
        return
    result["moved_above"] = True
    # Descend and push the knob
    down = Pose([x, y, z + 0.08], quat, frame)
    r = move_to_pose(down, 0.02, 200)
    if not r.ok:
        return
    result["pressed"] = True
    r = move_to_pose(Pose([x, y, z + 0.20], quat, frame), 0.02, 200)
    result["done"] = bool(r.ok)

try:
    run()
except Exception as e:
    result["error"] = str(e)
