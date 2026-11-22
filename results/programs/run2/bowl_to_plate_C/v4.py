def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]


base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "grasped": False, "placed": False}


def locate(queries, max_size=0.30):
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            ext = f.geometry.extents
            if max(float(ext[0]), float(ext[1])) <= max_size:
                return f
    return None


def go(x, y, z, tol=0.05, steps=400, retry=True):
    pose = Pose((x, y, z), downward, base_frame)
    r = move_to_pose(pose, tolerance=tol, max_steps=steps)
    if not r.ok and retry:
        r = move_to_pose(pose, tolerance=tol + 0.02, max_steps=steps)
    return r


bowl = locate(("bowl", "silver bowl", "metal bowl", "black bowl"))
plate = locate(("plate", "white plate", "round plate"))
if bowl is None or plate is None:
    result["error"] = "localization failed"
else:
    result["localized"] = True
    bc = bowl.geometry.pose.position
    pts = [p for p in bowl.point_cloud.points]
    zs = [float(p[2]) for p in pts]
    top_z = quantile(zs, 0.95)
    bottom_z = min(zs)
    plate_pts = plate.point_cloud.points
    pcx = quantile((p[0] for p in plate_pts), 0.5)
    pcy = quantile((p[1] for p in plate_pts), 0.5)
    plate_top = quantile((p[2] for p in plate_pts), 0.9)

    rim = [p for p in pts if float(p[2]) >= top_z - 0.015]
    if len(rim) < 3:
        rim = pts
    min_x = min(float(p[0]) for p in rim)
    near = [p for p in rim if float(p[0]) <= min_x + 0.02]
    gx = sum(float(p[0]) for p in near) / len(near) + 0.008
    gy = sum(float(p[1]) for p in near) / len(near)
    gz = top_z - 0.02

    open_gripper()
    r1 = go(gx, gy, top_z + 0.12)
    r2 = go(gx, gy, gz, tol=0.03)
    close_gripper()
    r3 = go(gx, gy, top_z + 0.15)
    result["grasped"] = bool(r2.ok or r3.ok)
    result["lift_ok"] = bool(r3.ok)

    dx = gx - float(bc[0])
    dy = gy - float(bc[1])
    tx = pcx + dx
    ty = pcy + dy
    r4 = go(tx, ty, top_z + 0.15, tol=0.06)
    rel_z = plate_top + (gz - bottom_z) + 0.04
    r5 = go(tx, ty, rel_z, tol=0.04)
    open_gripper()
    result["placed"] = bool(r4.ok and r5.ok)
    result["r4"] = bool(r4.ok)
    result["r5"] = bool(r5.ok)
    go(tx, ty, top_z + 0.15, tol=0.08, steps=150, retry=False)
