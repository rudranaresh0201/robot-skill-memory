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

result = {"localized": False, "grasped": False, "placed": False, "attempts": []}
base_frame = get_robot_state().base_frame
down = get_robot_state().end_effector_poses["primary"].quaternion_wxyz

bowl = locate(("black bowl", "akita black bowl", "bowl"))
stove = locate(("flat stove", "stove", "black stove burner"))
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

    open_gripper()
    grasp = None
    for sign in (-1.0, 1.0):
        ox = sign * (radius - 0.008)
        gx = bc[0] + ox
        gy = bc[1]
        gz = top_z - 0.012
        ok1 = go((gx, gy, gz + 0.12), down, base_frame, 0.03, 300)
        if not ok1:
            result["attempts"].append("pregrasp fail")
            continue
        go((gx, gy, gz + 0.05), down, base_frame, 0.02, 300)
        go((gx, gy, gz), down, base_frame, 0.02, 300)
        if near((gx, gy, gz), 0.04):
            result["attempts"].append("descend ok")
            grasp = (gx, gy, gz, ox)
            break
        result["attempts"].append("descend fail")
        go((gx, gy, gz + 0.12), down, base_frame, 0.04, 200)

    if grasp is None:
        result["notes"] = "approach failed"
    else:
        gx, gy, gz, ox = grasp
        close_gripper()
        go((gx, gy, gz + 0.18), down, base_frame, 0.03, 300)
        lifted = near((gx, gy, gz + 0.18), 0.05)
        result["grasped"] = lifted
        if lifted:
            tx = sc[0] + ox
            ty = sc[1]
            go((tx, ty, gz + 0.18), down, base_frame, 0.03, 400)
            rz = stove_top + (gz - bot_z) + 0.03
            go((tx, ty, max(rz, stove_top + 0.08)), down, base_frame, 0.03, 300)
            open_gripper()
            result["placed"] = True
            go((tx, ty, gz + 0.18), down, base_frame, 0.05, 150)
