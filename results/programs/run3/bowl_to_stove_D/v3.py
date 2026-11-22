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
    bc = (quantile([p[0] for p in pts], 0.5), quantile([p[1] for p in pts], 0.5), quantile([p[2] for p in pts], 0.5))
    top_z = quantile([p[2] for p in pts], 0.95)
    bot_z = min(p[2] for p in pts)
    spts = stove.point_cloud.points
    sc = (quantile([p[0] for p in spts], 0.5), quantile([p[1] for p in spts], 0.5))
    stove_top = quantile([p[2] for p in spts], 0.9)
    ext = bowl.geometry.extents
    radius = min(max(ext[0], ext[1]) / 2.0, 0.07)
    result["radius"] = radius

    open_gripper()
    cands = [
        (-(radius - 0.012), 0.0, 0.018),
        ((radius - 0.012), 0.0, 0.018),
    ]
    grasp = None
    for (ox, oy, depth) in cands:
        gx = bc[0] + ox
        gy = bc[1] + oy
        gz = top_z - depth
        r1 = move_to_pose(Pose((gx, gy, gz + 0.14), down, base_frame), tolerance=0.04, max_steps=200)
        if not r1.ok and not near((gx, gy, gz + 0.14), 0.04):
            result["attempts"].append("pregrasp fail")
            continue
        move_to_pose(Pose((gx, gy, gz + 0.04), down, base_frame), tolerance=0.04, max_steps=100)
        move_to_pose(Pose((gx, gy, gz), down, base_frame), tolerance=0.05, max_steps=100)
        if near((gx, gy, gz), 0.03):
            result["attempts"].append("descend ok")
            grasp = (gx, gy, gz, ox, oy)
            break
        result["attempts"].append("descend fail")
        move_to_pose(Pose((gx, gy, gz + 0.14), down, base_frame), tolerance=0.05, max_steps=100)

    if grasp is None:
        result["notes"] = "approach failed"
    else:
        gx, gy, gz, ox, oy = grasp
        close_gripper()
        move_to_pose(Pose((gx, gy, gz + 0.18), down, base_frame), tolerance=0.04, max_steps=150)
        lifted = near((gx, gy, gz + 0.18), 0.05)
        result["grasped"] = lifted
        if lifted:
            tx = sc[0] + ox
            ty = sc[1] + oy
            move_to_pose(Pose((tx, ty, gz + 0.18), down, base_frame), tolerance=0.04, max_steps=250)
            rz = stove_top + (gz - bot_z) + 0.03
            move_to_pose(Pose((tx, ty, max(rz, stove_top + 0.08)), down, base_frame), tolerance=0.04, max_steps=150)
            open_gripper()
            result["placed"] = True
            move_to_pose(Pose((tx, ty, gz + 0.18), down, base_frame), tolerance=0.05, max_steps=100)
