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
    spts = stove.point_cloud.points
    sc = (quantile([p[0] for p in spts], 0.5), quantile([p[1] for p in spts], 0.5))
    stove_top = quantile([p[2] for p in spts], 0.9)
    ext = bowl.geometry.extents
    radius = min(max(ext[0], ext[1]) / 2.0, 0.07)
    result["radius"] = radius

    open_gripper()
    # Candidate rim grasps: (dx, dy, depth below rim top)
    cands = [
        (radius - 0.015, 0.0, 0.012),
        (radius - 0.015, 0.0, 0.0),
        (-(radius - 0.015), 0.0, 0.012),
    ]
    grasp = None
    for (ox, oy, depth) in cands:
        gx = bc[0] + ox
        gy = bc[1] + oy
        gz = top_z - depth
        r1 = move_to_pose(Pose((gx, gy, gz + 0.14), down, base_frame), tolerance=0.03, max_steps=200)
        if not r1.ok:
            result["attempts"].append("pregrasp fail")
            continue
        r2 = move_to_pose(Pose((gx, gy, gz + 0.04), down, base_frame), tolerance=0.03, max_steps=150)
        r3 = move_to_pose(Pose((gx, gy, gz), down, base_frame), tolerance=0.03, max_steps=150) if r2.ok else r2
        result["attempts"].append("descend ok" if r3.ok else "descend fail")
        if r3.ok:
            grasp = (gx, gy, gz, ox, oy)
            break
        move_to_pose(Pose((gx, gy, gz + 0.14), down, base_frame), tolerance=0.04, max_steps=150)

    if grasp is None:
        result["notes"] = "approach failed"
    else:
        gx, gy, gz, ox, oy = grasp
        close_gripper()
        lifted = move_to_pose(Pose((gx, gy, gz + 0.18), down, base_frame), tolerance=0.03, max_steps=200).ok
        result["grasped"] = lifted
        if lifted:
            tx = sc[0] + ox
            ty = sc[1] + oy
            if move_to_pose(Pose((tx, ty, gz + 0.18), down, base_frame), tolerance=0.03, max_steps=300).ok:
                rz = stove_top + (gz - min(p[2] for p in pts)) + 0.03
                if move_to_pose(Pose((tx, ty, max(rz, stove_top + 0.08)), down, base_frame), tolerance=0.03, max_steps=200).ok:
                    open_gripper()
                    result["placed"] = True
                    move_to_pose(Pose((tx, ty, gz + 0.18), down, base_frame), tolerance=0.04, max_steps=150)
