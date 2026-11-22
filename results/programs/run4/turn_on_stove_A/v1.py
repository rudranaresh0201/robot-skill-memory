import math

result = {"localized": False, "moved_above": False, "pressed": False}

def near_ok(r):
    return r.ok or (r.error is not None and r.error.details.get("joint_error_rad", 9) < 0.05)

loc = localize_object("flat stove")
if not loc.ok:
    loc = localize_object("stove burner knob")
result["loc_ok"] = loc.ok

if loc.ok:
    p = loc.geometry.pose.position
    x, y, z = float(p[0]), float(p[1]), float(p[2])
    result["stove_pos"] = [x, y, z]
    result["localized"] = True
    st = get_robot_state()
    ee = st.end_effector_poses if hasattr(st, "end_effector_poses") else None
    quat = [0.0, 1.0, 0.0, 0.0]
    close_gripper()
    # go up first
    cur = get_robot_state()
    try:
        cp = list(cur.ee_poses["primary"].position)
    except Exception:
        cp = None
    r = move_to_pose(Pose([x, y, 0.32], quat, "robot_base"))
    result["moved_above"] = near_ok(r)
    if result["moved_above"]:
        # knob is near front of stove; try pressing/turning down onto the stove top
        r2 = move_to_pose(Pose([x, y, max(z + 0.03, 0.10)], quat, "robot_base"))
        result["descended"] = near_ok(r2)
        r3 = move_to_pose(Pose([x, y, 0.32], quat, "robot_base"))
        result["pressed"] = near_ok(r2)
