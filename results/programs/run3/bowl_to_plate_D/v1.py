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

result = {"localized": False, "grasped": False, "placed": False}

base_frame = get_robot_state().base_frame
down = get_robot_state().end_effector_poses["primary"].quaternion_wxyz

bowl = locate(("black bowl", "akita black bowl", "bowl"))
plate = locate(("plate", "white plate", "round plate"))
if bowl is None or plate is None:
    result["error"] = "localization failed"
else:
    result["localized"] = True
    bpts = bowl.point_cloud.points
    bc = (quantile((p[0] for p in bpts), 0.5), quantile((p[1] for p in bpts), 0.5))
    btop = quantile((p[2] for p in bpts), 0.95)
    ppts = plate.point_cloud.points
    pc = (quantile((p[0] for p in ppts), 0.5), quantile((p[1] for p in ppts), 0.5))
    ptop = quantile((p[2] for p in ppts), 0.9)

    # grasp the bowl rim: offset in x toward the near edge using extents
    ext = bowl.geometry.extents
    radius = max(ext[0], ext[1]) / 2.0
    gx, gy = bc[0] - radius * 0.8, bc[1]
    gz = btop - 0.02

    open_gripper()
    ok = move_to_pose(Pose((gx, gy, gz + 0.12), down, base_frame), tolerance=0.02, max_steps=250).ok
    if ok:
        ok = move_to_pose(Pose((gx, gy, gz), down, base_frame), tolerance=0.015, max_steps=250).ok
    result["approach_ok"] = ok
    if ok:
        close_gripper()
        result["grasped"] = True
        lift = move_to_pose(Pose((gx, gy, gz + 0.18), down, base_frame), tolerance=0.02, max_steps=250)
        if lift.ok:
            # keep same grasp offset relative to bowl center
            tx, ty = pc[0] - radius * 0.8, pc[1]
            t = move_to_pose(Pose((tx, ty, gz + 0.18), down, base_frame), tolerance=0.02, max_steps=300)
            if t.ok:
                d = move_to_pose(Pose((tx, ty, ptop + 0.06 + (btop - gz) * 0.0 + 0.03), down, base_frame), tolerance=0.02, max_steps=250)
                if d.ok:
                    open_gripper()
                    result["placed"] = True
                    move_to_pose(Pose((tx, ty, gz + 0.18), down, base_frame), tolerance=0.03, max_steps=250)
