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
    pc = plate.geometry.pose.position
    plate_top = quantile((p[2] for p in plate.point_cloud.points), 0.9)

    # rim grasp point: top-most points, take the one on the near (min x) side
    rim = [p for p in pts if float(p[2]) >= top_z - 0.015]
    if len(rim) < 3:
        rim = pts
    min_x = min(float(p[0]) for p in rim)
    near = [p for p in rim if float(p[0]) <= min_x + 0.02]
    gx = sum(float(p[0]) for p in near) / len(near) + 0.008
    gy = sum(float(p[1]) for p in near) / len(near)
    gz = top_z - 0.02

    open_gripper()
    r1 = move_to_pose(Pose((gx, gy, top_z + 0.12), downward, base_frame), tolerance=0.04, max_steps=300)
    r2 = move_to_pose(Pose((gx, gy, gz), downward, base_frame), tolerance=0.04, max_steps=300)
    close_gripper()
    r3 = move_to_pose(Pose((gx, gy, top_z + 0.15), downward, base_frame), tolerance=0.04, max_steps=300)
    result["grasped"] = bool(r2.ok or r3.ok)
    result["lift_ok"] = bool(r3.ok)

    # keep bowl center over plate center
    dx = gx - float(bc[0])
    dy = gy - float(bc[1])
    tx = float(pc[0]) + dx
    ty = float(pc[1]) + dy
    r4 = move_to_pose(Pose((tx, ty, top_z + 0.15), downward, base_frame), tolerance=0.04, max_steps=300)
    rel_z = plate_top + (gz - bottom_z) + 0.03
    r5 = move_to_pose(Pose((tx, ty, rel_z), downward, base_frame), tolerance=0.04, max_steps=300)
    open_gripper()
    r6 = move_to_pose(Pose((tx, ty, top_z + 0.15), downward, base_frame), tolerance=0.05, max_steps=200)
    result["placed"] = bool(r4.ok and r5.ok)
