def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]

def locate(queries, camera_name="agentview"):
    base_frame = get_robot_state().base_frame
    for q in queries:
        f = localize_object(q, camera_name=camera_name, target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None

def near(target, tol):
    p = get_robot_state().end_effector_poses["primary"].position
    return (abs(p[0] - target[0]) < tol and abs(p[1] - target[1]) < tol
            and abs(p[2] - target[2]) < tol)

def go(pos, quat, frame, tol, steps):
    r = move_to_pose(Pose(pos, quat, frame), tolerance=tol, max_steps=steps)
    return r.ok or near(pos, tol * 2.0)

def travel(target, quat, frame, seg=0.10, tol=0.03, steps=200):
    """Move laterally in short segments to keep joint-space interpolation small."""
    p = get_robot_state().end_effector_poses["primary"].position
    sx, sy, sz = float(p[0]), float(p[1]), float(p[2])
    dist = math.sqrt((target[0] - sx) ** 2 + (target[1] - sy) ** 2 + (target[2] - sz) ** 2)
    n = max(1, int(math.ceil(dist / seg)))
    for i in range(1, n + 1):
        f = i / float(n)
        wp = (sx + (target[0] - sx) * f, sy + (target[1] - sy) * f, sz + (target[2] - sz) * f)
        if not go(wp, quat, frame, tol, steps):
            if not go(wp, quat, frame, tol * 1.5, steps):
                return False
    return True

result = {"localized": False, "grasped": False, "placed": False, "attempts": []}
base_frame = get_robot_state().base_frame
down = get_robot_state().end_effector_poses["primary"].quaternion_wxyz

bowl = locate(("black bowl", "akita black bowl", "bowl"))
stove = locate(("black stove burner", "flat stove", "stove"))
if bowl is None or stove is None:
    result["notes"] = "localization failed"
else:
    result["localized"] = True
    pts = bowl.point_cloud.points
    bc = (quantile([p[0] for p in pts], 0.5), quantile([p[1] for p in pts], 0.5))
    top_z = quantile([p[2] for p in pts], 0.95)
    bot_z = min(p[2] for p in pts)
    spts = stove.point_cloud.points
    sc = (quantile([p[0] for p in spts], 0.5), quantile([p[1] for p in spts], 0.5))
    stove_top = quantile([p[2] for p in spts], 0.9)
    ext = bowl.geometry.extents
    radius = min(max(ext[0], ext[1]) / 2.0, 0.07)
    result["radius"] = radius
    result["stove_center"] = sc

    open_gripper()
    grasp = None
    for sign in (-1.0, 1.0):
        for dz in (0.0, 0.015):
            ox = sign * (radius - 0.012)
            gx = bc[0] + ox
            gy = bc[1]
            gz = top_z + dz - 0.004
            if not go((gx, gy, gz + 0.12), down, base_frame, 0.03, 300):
                result["attempts"].append("pregrasp fail")
                continue
            go((gx, gy, gz + 0.04), down, base_frame, 0.02, 200)
            go((gx, gy, gz), down, base_frame, 0.02, 200)
            if near((gx, gy, gz), 0.035):
                result["attempts"].append("descend ok")
                grasp = (gx, gy, gz, ox)
                break
            result["attempts"].append("descend fail")
            go((gx, gy, gz + 0.12), down, base_frame, 0.04, 200)
        if grasp is not None:
            break

    if grasp is None:
        result["notes"] = "approach failed"
    else:
        gx, gy, gz, ox = grasp
        close_gripper()
        carry_z = max(gz + 0.22, stove_top + 0.25)
        lifted = go((gx, gy, carry_z), down, base_frame, 0.03, 300)
        result["grasped"] = bool(lifted)
        if lifted:
            tx = sc[0] + ox
            ty = sc[1]
            arrived = travel((tx, ty, carry_z), down, base_frame)
            result["arrived"] = bool(arrived)
            if arrived:
                rz = max(stove_top + (gz - bot_z) + 0.03, stove_top + 0.06)
                go((tx, ty, rz), down, base_frame, 0.03, 300)
                open_gripper()
                result["placed"] = near((tx, ty, rz), 0.06)
                go((tx, ty, carry_z), down, base_frame, 0.05, 150)
            else:
                result["notes"] = "transport failed"
