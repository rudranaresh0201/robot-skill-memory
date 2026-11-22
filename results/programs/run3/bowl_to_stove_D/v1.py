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

    # Grasp bowl rim: offset toward the +x side of the bowl rim from center
    ext = bowl.geometry.extents
    radius = max(ext[0], ext[1]) / 2.0
    gx = bc[0] + radius - 0.012
    gy = bc[1]
    gz = top_z - 0.015

    open_gripper()
    ok = move_to_pose(Pose((gx, gy, gz + 0.12), down, base_frame), tolerance=0.02, max_steps=250).ok
    if ok:
        ok = move_to_pose(Pose((gx, gy, gz), down, base_frame), tolerance=0.015, max_steps=250).ok
    if ok:
        close_gripper()
        lifted = move_to_pose(Pose((gx, gy, gz + 0.18), down, base_frame), tolerance=0.02, max_steps=250).ok
        result["grasped"] = lifted
        if lifted:
            dx = gx - bc[0]
            dy = gy - bc[1]
            tx = sc[0] + dx
            ty = sc[1] + dy
            if move_to_pose(Pose((tx, ty, gz + 0.18), down, base_frame), tolerance=0.02, max_steps=300).ok:
                rz = stove_top + (gz - bc[2] + (bc[2] - min(p[2] for p in pts))) + 0.03
                if move_to_pose(Pose((tx, ty, max(rz, stove_top + 0.08)), down, base_frame), tolerance=0.02, max_steps=250).ok:
                    open_gripper()
                    result["placed"] = True
                    move_to_pose(Pose((tx, ty, gz + 0.18), down, base_frame), tolerance=0.03, max_steps=250)
    else:
        result["notes"] = "approach failed"
