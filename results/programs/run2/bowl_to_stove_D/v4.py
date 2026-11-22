import math

def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]

def locate(queries, camera_name="agentview"):
    base_frame = get_robot_state().base_frame
    for q in queries:
        found = localize_object(q, camera_name=camera_name, target_frame=base_frame)
        if found.ok and len(found.point_cloud.points) >= 20:
            return found
    return None

result = {"localized": False, "grasped": False, "placed": False, "notes": ""}
base_frame = get_robot_state().base_frame
quat = get_robot_state().end_effector_poses["primary"].quaternion_wxyz

def mv1(pos, tol, steps):
    for attempt in range(2):
        r = move_to_pose(Pose(pos, quat, base_frame),
                         tolerance=tol + 0.01 * attempt,
                         max_steps=steps + 100 * attempt)
        if r.ok:
            return True
    return False

def mv(pos, tol=0.02, steps=200, stride=0.10):
    """Move in short Cartesian increments so IK/servo stays well-conditioned."""
    cur = get_robot_state().end_effector_poses["primary"].position
    cx, cy, cz = float(cur[0]), float(cur[1]), float(cur[2])
    dx, dy, dz = pos[0] - cx, pos[1] - cy, pos[2] - cz
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    n = max(1, int(math.ceil(dist / stride)))
    ok = True
    for i in range(1, n + 1):
        f = i / n
        p = (cx + dx * f, cy + dy * f, cz + dz * f)
        t = tol if i == n else tol + 0.01
        if not mv1(p, t, steps):
            ok = False
            break
    return ok

bowl = locate(("black bowl", "akita black bowl", "bowl"))
stove = locate(("flat black stove with burner", "flat stove", "stove", "cooktop"))
if bowl is None or stove is None:
    result["notes"] = "localization failed"
else:
    result["localized"] = True
    pts = bowl.point_cloud.points
    bx = quantile((p[0] for p in pts), 0.5)
    by = quantile((p[1] for p in pts), 0.5)
    btop = quantile((p[2] for p in pts), 0.95)
    bz0 = quantile((p[2] for p in pts), 0.5)
    spts = stove.point_cloud.points
    sx = quantile((p[0] for p in spts), 0.5)
    sy = quantile((p[1] for p in spts), 0.5)
    sz = quantile((p[2] for p in spts), 0.95)
    result["bowl"] = [bx, by, btop]
    result["stove"] = [sx, sy, sz]
    try:
        open_gripper()
        ext = bowl.geometry.extents
        rim_off = max(float(ext[0]), float(ext[1])) / 2.0 - 0.012
        gx, gy, gz = bx, by + rim_off, btop - 0.015
        safe_z = btop + 0.15
        ok1 = mv((gx, gy, safe_z), 0.025)
        ok2 = mv((gx, gy, gz), 0.015, 200, 0.05)
        close_gripper()
        ok3 = mv((gx, gy, safe_z), 0.025, 200, 0.06)
        lifted = False
        verify = locate(("black bowl", "akita black bowl", "bowl"))
        if verify is not None:
            vz = quantile((p[2] for p in verify.point_cloud.points), 0.5)
            lifted = vz - bz0 > 0.03
        result["grasped"] = bool(ok2 and (ok3 or lifted))
        result["grasp_steps"] = [ok1, ok2, ok3, lifted]
        ok4 = mv((sx, sy + rim_off, safe_z), 0.03, 250)
        ok5 = mv((sx, sy + rim_off, sz + 0.09), 0.02, 200, 0.05)
        open_gripper()
        mv((sx, sy + rim_off, sz + 0.2), 0.04, 200)
        result["place_steps"] = [ok4, ok5]
        result["placed"] = bool(result["grasped"] and ok5)
    except Exception as e:
        result["notes"] = str(e)
