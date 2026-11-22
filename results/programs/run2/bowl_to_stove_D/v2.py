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

def mv(pos, tol=0.02, steps=300):
    """Move with retries; each retry continues from wherever the arm ended up."""
    last = None
    for attempt in range(3):
        last = move_to_pose(Pose(pos, quat, base_frame),
                            tolerance=tol + 0.01 * attempt,
                            max_steps=steps + 100 * attempt)
        if last.ok:
            return True
    return False

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
    spts = stove.point_cloud.points
    sx = quantile((p[0] for p in spts), 0.5)
    sy = quantile((p[1] for p in spts), 0.5)
    sz = quantile((p[2] for p in spts), 0.9)
    result["bowl"] = [bx, by, btop]
    result["stove"] = [sx, sy, sz]
    try:
        open_gripper()
        ext = bowl.geometry.extents
        rim_off = max(float(ext[0]), float(ext[1])) / 2.0 - 0.015
        # grasp the rim on the side nearer the robot base (smaller x)
        gx, gy, gz = bx - rim_off, by, btop - 0.025
        ok1 = mv((gx, gy, gz + 0.15), 0.025, 300)
        ok2 = mv((gx, gy, gz), 0.015, 300)
        close_gripper()
        ok3 = mv((gx, gy, gz + 0.18), 0.025, 300)
        verify = locate(("black bowl", "akita black bowl", "bowl"))
        lifted = False
        if verify is not None:
            vz = quantile((p[2] for p in verify.point_cloud.points), 0.5)
            lifted = vz - quantile((p[2] for p in pts), 0.5) > 0.03
        result["grasped"] = bool(ok2 and ok3 and lifted)
        result["grasp_steps"] = [ok1, ok2, ok3, lifted]
        # transport, keeping same offset so the bowl center is over the stove
        mv((sx - rim_off, sy, gz + 0.2), 0.03, 350)
        mv((sx - rim_off, sy, sz + 0.1), 0.02, 300)
        open_gripper()
        mv((sx - rim_off, sy, sz + 0.22), 0.04, 250)
        result["placed"] = bool(result["grasped"])
    except Exception as e:
        result["notes"] = str(e)
