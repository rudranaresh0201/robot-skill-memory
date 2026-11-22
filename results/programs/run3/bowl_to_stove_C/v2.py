def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]


def locate(queries, camera_name="agentview"):
    base = get_robot_state().base_frame
    for q in queries:
        f = localize_object(q, camera_name=camera_name, target_frame=base)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None


def go(x, y, z, quat, frame, tol=0.025, steps=450):
    r = move_to_pose(Pose((x, y, z), quat, frame), tolerance=tol, max_steps=steps)
    if r.ok:
        return True
    # one retry from wherever the arm stopped (slow arm, not necessarily blocked)
    r = move_to_pose(Pose((x, y, z), quat, frame), tolerance=tol + 0.01, max_steps=300)
    return r.ok


result = {"localized": False, "grasped": False, "placed": False}
base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz

bowl = locate(("black bowl", "akita black bowl", "bowl"))
stove = locate(("flat stove", "black stove", "stove"))
if bowl is None or stove is None:
    result["error"] = "localization failed"
else:
    result["localized"] = True
    bc = bowl.geometry.pose.position
    bzs = [p[2] for p in bowl.point_cloud.points]
    top_z = quantile(bzs, 0.97)
    bot_z = quantile(bzs, 0.03)
    sc = stove.geometry.pose.position
    stove_z = quantile((p[2] for p in stove.point_cloud.points), 0.9)

    ext = bowl.geometry.extents
    radius = min(float(ext[0]), float(ext[1])) / 2.0
    # grip the rim on the y side (fingers separate along y, straddle the wall)
    sign = -1.0 if sc[1] > bc[1] else 1.0
    dy = sign * max(radius - 0.012, 0.02)
    gx = bc[0]
    gy = bc[1] + dy
    grasp_z = top_z - 0.02
    result["rim"] = [gx, gy, grasp_z]

    open_gripper()
    ok = go(gx, gy, top_z + 0.12, downward, base_frame)
    if ok:
        ok = go(gx, gy, grasp_z, downward, base_frame, tol=0.015)
    result["approach_ok"] = ok
    if ok:
        close_gripper()
        lift_z = top_z + 0.22
        lifted = go(gx, gy, lift_z, downward, base_frame)
        result["grasped"] = lifted
        if lifted:
            tx = sc[0]
            ty = sc[1] + dy
            moved = go(tx, ty, lift_z, downward, base_frame, tol=0.03)
            result["moved"] = moved
            if moved:
                rel_z = stove_z + (grasp_z - bot_z) + 0.03
                rel = go(tx, ty, rel_z, downward, base_frame, tol=0.02)
                open_gripper()
                result["placed"] = rel
                go(tx, ty, lift_z, downward, base_frame, tol=0.04, steps=250)
