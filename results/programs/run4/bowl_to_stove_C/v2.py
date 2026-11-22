def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]


def near_ok(r):
    return r.ok or (r.error is not None and r.error.details.get("joint_error_rad", 9) < 0.05)


base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "grasped": False, "placed": False}


def locate(queries):
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None


bowl = locate(("black bowl", "bowl"))
stove = locate(("flat stove", "stove", "black stove burner"))
if bowl is None or stove is None:
    result["error"] = "localization failed"
else:
    result["localized"] = True
    bc = bowl.geometry.pose.position
    bz_top = quantile((p[2] for p in bowl.point_cloud.points), 0.9)
    sc = stove.geometry.pose.position
    stove_top = quantile((p[2] for p in stove.point_cloud.points), 0.9)
    SAFE = 0.32
    ext = bowl.geometry.extents
    radius = max(float(ext[0]), float(ext[1])) / 2.0
    gz = bz_top - 0.02
    open_gripper()
    chosen = None
    for sign in (1.0, -1.0):
        # rim contact along y so the fingers (separating along y) straddle the wall
        gx, gy = bc[0], bc[1] + sign * (radius - 0.012)
        r = move_to_pose(Pose((gx, gy, SAFE), downward, base_frame), tolerance=0.02, max_steps=250)
        ok1 = near_ok(r)
        if not ok1:
            continue
        r = move_to_pose(Pose((gx, gy, gz), downward, base_frame), tolerance=0.02, max_steps=150)
        if near_ok(r):
            chosen = (gx, gy, sign)
            break
        move_to_pose(Pose((gx, gy, SAFE), downward, base_frame), tolerance=0.03, max_steps=150)
    result["descended"] = chosen is not None
    if chosen is not None:
        gx, gy, sign = chosen
        close_gripper()
        r = move_to_pose(Pose((gx, gy, SAFE), downward, base_frame), tolerance=0.02, max_steps=250)
        result["lifted"] = near_ok(r)
        after = locate(("black bowl", "bowl"))
        if after is not None:
            result["grasped"] = quantile((p[2] for p in after.point_cloud.points), 0.5) - quantile((p[2] for p in bowl.point_cloud.points), 0.5) > 0.03
        tx, ty = sc[0], sc[1] + sign * (radius - 0.012)
        r = move_to_pose(Pose((tx, ty, SAFE), downward, base_frame), tolerance=0.02, max_steps=300)
        if near_ok(r):
            release_z = stove_top + (gz - (bc[2] - float(ext[2]) / 2.0)) + 0.03
            r = move_to_pose(Pose((tx, ty, max(release_z, stove_top + 0.08)), downward, base_frame), tolerance=0.02, max_steps=250)
            if near_ok(r):
                open_gripper()
                result["placed"] = True
                move_to_pose(Pose((tx, ty, SAFE), downward, base_frame), tolerance=0.03, max_steps=250)
