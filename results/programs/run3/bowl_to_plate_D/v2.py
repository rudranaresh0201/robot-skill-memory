def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]

def locate(queries, camera_name="agentview", max_ext=None):
    base_frame = get_robot_state().base_frame
    for q in queries:
        f = localize_object(q, camera_name=camera_name, target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            if max_ext is not None:
                e = f.geometry.extents
                if max(e[0], e[1]) > max_ext:
                    continue
            return f
    return None

result = {"localized": False, "grasped": False, "placed": False}

base_frame = get_robot_state().base_frame
down = get_robot_state().end_effector_poses["primary"].quaternion_wxyz

# The bowl is silver/gray patterned, not black (black = wine bottle), so avoid "black" prompts
bowl = locate(("bowl", "silver bowl", "gray metal bowl"), max_ext=0.3)
plate = locate(("plate", "red rimmed plate", "round plate"), max_ext=0.4)
if bowl is None or plate is None:
    result["error"] = "localization failed"
else:
    result["localized"] = True
    bpts = bowl.point_cloud.points
    bc = (quantile((p[0] for p in bpts), 0.5), quantile((p[1] for p in bpts), 0.5))
    btop = quantile((p[2] for p in bpts), 0.95)
    bbot = quantile((p[2] for p in bpts), 0.05)
    ppts = plate.point_cloud.points
    pc = (quantile((p[0] for p in ppts), 0.5), quantile((p[1] for p in ppts), 0.5))
    ptop = quantile((p[2] for p in ppts), 0.9)

    ext = bowl.geometry.extents
    radius = max(ext[0], ext[1]) / 2.0
    result["bowl_center"] = bc
    result["bowl_radius"] = radius
    result["plate_center"] = pc
    off = radius * 0.85
    gx, gy = bc[0] - off, bc[1]
    gz = btop - 0.02
    carry = max(gz + 0.18, ptop + (gz - bbot) + 0.15)

    open_gripper()
    ok = move_to_pose(Pose((gx, gy, gz + 0.12), down, base_frame), tolerance=0.02, max_steps=250).ok
    if ok:
        ok = move_to_pose(Pose((gx, gy, gz), down, base_frame), tolerance=0.015, max_steps=250).ok
    result["approach_ok"] = ok
    if ok:
        close_gripper()
        result["grasped"] = True
        lift = move_to_pose(Pose((gx, gy, carry), down, base_frame), tolerance=0.02, max_steps=250)
        if lift.ok:
            tx, ty = pc[0] - off, pc[1]
            t = move_to_pose(Pose((tx, ty, carry), down, base_frame), tolerance=0.02, max_steps=300)
            if t.ok:
                rz = ptop + (gz - bbot) + 0.03
                d = move_to_pose(Pose((tx, ty, rz), down, base_frame), tolerance=0.02, max_steps=250)
                if d.ok:
                    open_gripper()
                    result["placed"] = True
                    move_to_pose(Pose((tx, ty, carry), down, base_frame), tolerance=0.03, max_steps=250)
