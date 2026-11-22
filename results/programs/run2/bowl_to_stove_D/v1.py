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

result = {"localized": False, "grasped": False, "placed": False}
base_frame = get_robot_state().base_frame
quat = get_robot_state().end_effector_poses["primary"].quaternion_wxyz

bowl = locate(("black bowl", "akita black bowl", "bowl"))
stove = locate(("flat stove", "black stove", "stove", "cooktop"))
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
    try:
        open_gripper()
        # grasp the bowl rim: offset toward the near edge using extents
        ext = bowl.geometry.extents
        rim_off = max(float(ext[0]), float(ext[1])) / 2.0 - 0.015
        gx, gy, gz = bx - rim_off, by, btop - 0.02
        r1 = move_to_pose(Pose((gx, gy, gz + 0.14), quat, base_frame), tolerance=0.02, max_steps=250)
        r2 = move_to_pose(Pose((gx, gy, gz), quat, base_frame), tolerance=0.015, max_steps=250)
        close_gripper()
        r3 = move_to_pose(Pose((gx, gy, gz + 0.18), quat, base_frame), tolerance=0.02, max_steps=250)
        result["grasped"] = bool(r1.ok and r2.ok)
        # transport; keep same grasp offset so bowl centers over stove
        move_to_pose(Pose((sx - rim_off, sy, gz + 0.18), quat, base_frame), tolerance=0.02, max_steps=300)
        move_to_pose(Pose((sx - rim_off, sy, sz + 0.09), quat, base_frame), tolerance=0.02, max_steps=250)
        open_gripper()
        move_to_pose(Pose((sx - rim_off, sy, sz + 0.2), quat, base_frame), tolerance=0.03, max_steps=250)
        result["placed"] = True
    except Exception as e:
        result["notes"] = str(e)
