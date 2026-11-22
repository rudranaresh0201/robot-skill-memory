def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]


def locate_checked(queries, table_z, max_h, camera_name="agentview"):
    base = get_robot_state().base_frame
    for q in queries:
        f = localize_object(q, camera_name=camera_name, target_frame=base)
        if f.ok and len(f.point_cloud.points) >= 20:
            top = quantile((p[2] for p in f.point_cloud.points), 0.9)
            if top - table_z < max_h:
                return f
    return None


def go(x, y, z, quat, frame, tol=0.025, steps=400, curobo=False):
    strat = MotionStrategy(trajectory_planner="curobo") if curobo else None
    r = move_to_pose(Pose((x, y, z), quat, frame), tolerance=tol, max_steps=steps, strategy=strat)
    return r.ok


result = {"localized": False, "grasped": False, "placed": False}
base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz

bf = localize_object("black bowl", camera_name="agentview", target_frame=base_frame)
if not bf.ok or len(bf.point_cloud.points) < 20:
    bf = localize_object("bowl", camera_name="agentview", target_frame=base_frame)
if not bf.ok or len(bf.point_cloud.points) < 20:
    result["error"] = "bowl not found"
else:
    bc = bf.geometry.pose.position
    bzs = [p[2] for p in bf.point_cloud.points]
    top_z = quantile(bzs, 0.97)
    bot_z = quantile(bzs, 0.03)
    stove = locate_checked(("flat stove with burner", "gray cooktop stove", "flat stove", "stove"),
                           bot_z, 0.15)
    if stove is None:
        result["error"] = "stove not found"
    else:
        result["localized"] = True
        sc = stove.geometry.pose.position
        stove_z = quantile((p[2] for p in stove.point_cloud.points), 0.9)
        result["stove"] = [sc[0], sc[1], stove_z]
        ext = bf.geometry.extents
        radius = min(float(ext[0]), float(ext[1])) / 2.0
        sign = -1.0 if sc[1] > bc[1] else 1.0
        dy = sign * max(radius - 0.012, 0.02)
        gx = bc[0]
        gy = bc[1] + dy
        grasp_z = top_z - 0.02

        open_gripper()
        ok = go(gx, gy, top_z + 0.12, downward, base_frame)
        if ok:
            ok = go(gx, gy, grasp_z, downward, base_frame, tol=0.015)
        result["approach_ok"] = ok
        if ok:
            close_gripper()
            lift_z = top_z + 0.28
            lifted = go(gx, gy, lift_z, downward, base_frame)
            result["grasped"] = lifted
            if lifted:
                tx = sc[0]
                ty = sc[1] + dy
                mid_x = 0.5 * (gx + tx)
                mid_y = 0.5 * (gy + ty)
                go(mid_x, mid_y, lift_z, downward, base_frame, tol=0.04, steps=300)
                moved = go(tx, ty, lift_z, downward, base_frame, tol=0.03, steps=300)
                if not moved:
                    moved = go(tx, ty, lift_z, downward, base_frame, tol=0.04, steps=300, curobo=True)
                result["moved"] = moved
                if moved:
                    rel_z = stove_z + (grasp_z - bot_z) + 0.03
                    rel = go(tx, ty, rel_z, downward, base_frame, tol=0.02)
                    open_gripper()
                    result["placed"] = rel
                    go(tx, ty, lift_z, downward, base_frame, tol=0.04, steps=250)
