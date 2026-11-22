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

def goto(pos, quat, frame, tol=0.025, steps=300):
    r = move_to_pose(Pose(pos, quat, frame), tolerance=tol, max_steps=steps)
    if r.ok:
        return True
    # retry once from the partially converged state with a looser tolerance
    r2 = move_to_pose(Pose(pos, quat, frame), tolerance=0.05, max_steps=steps)
    return r2.ok

result = {"localized": False, "grasped": False, "placed": False}

base_frame = get_robot_state().base_frame
down = get_robot_state().end_effector_poses["primary"].quaternion_wxyz

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
    result["plate_center"] = pc
    off = radius * 0.85
    gx, gy = bc[0] - off, bc[1]
    gz = btop - 0.025
    height = gz - bbot
    carry = max(gz + 0.2, ptop + height + 0.18)

    open_gripper()
    ok = goto((gx, gy, gz + 0.12), down, base_frame, 0.02, 250)
    if ok:
        ok = goto((gx, gy, gz), down, base_frame, 0.015, 250)
    result["approach_ok"] = ok
    if ok:
        close_gripper()
        result["grasped"] = True
        lift_ok = goto((gx, gy, carry), down, base_frame, 0.025, 300)
        result["lift_ok"] = lift_ok
        if lift_ok:
            tx, ty = pc[0] - off, pc[1]
            t_ok = goto((tx, ty, carry), down, base_frame, 0.03, 400)
            result["transport_ok"] = t_ok
            if t_ok:
                rz = ptop + height + 0.04
                d_ok = goto((tx, ty, rz), down, base_frame, 0.03, 300)
                result["descend_ok"] = d_ok
                if d_ok:
                    open_gripper()
                    result["placed"] = True
                    goto((tx, ty, carry), down, base_frame, 0.05, 250)
