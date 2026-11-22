def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]


def locate(queries, camera_name="agentview"):
    base = get_robot_state().base_frame
    for q in queries:
        f = localize_object(q, camera_name=camera_name, target_frame=base)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None


result = {"localized": False, "grasped": False, "placed": False}
base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz

bowl = locate(("black bowl", "akita black bowl", "bowl"))
stove = locate(("flat stove", "black stove", "stove"))
if bowl is None or stove is None:
    result["error"] = "localization failed"
else:
    result["localized"] = True
    bc = bowl.geometry.pose.position
    bzs = [p[2] for p in bowl.point_cloud.points]
    top_z = quantile(bzs, 0.95)
    sc = stove.geometry.pose.position
    stove_z = quantile((p[2] for p in stove.point_cloud.points), 0.9)

    # grasp the bowl rim: offset toward the nearest rim along x using extents
    ext = bowl.geometry.extents
    rim_x = bc[0] + max(float(e) for e in ext[:2]) / 2.0 - 0.01
    grasp_z = top_z - 0.015
    open_gripper()
    ok = move_to_pose(Pose((rim_x, bc[1], top_z + 0.12), downward, base_frame),
                      tolerance=0.02, max_steps=250).ok
    if ok:
        ok = move_to_pose(Pose((rim_x, bc[1], grasp_z), downward, base_frame),
                          tolerance=0.015, max_steps=250).ok
    result["approach_ok"] = ok
    if ok:
        close_gripper()
        lift_z = top_z + 0.18
        lifted = move_to_pose(Pose((rim_x, bc[1], lift_z), downward, base_frame),
                              tolerance=0.02, max_steps=250).ok
        result["grasped"] = lifted
        if lifted:
            # bowl center offset from the gripping point
            dx = rim_x - bc[0]
            tx = sc[0] + dx
            ty = sc[1]
            moved = move_to_pose(Pose((tx, ty, lift_z), downward, base_frame),
                                 tolerance=0.02, max_steps=300).ok
            if moved:
                rel = move_to_pose(Pose((tx, ty, stove_z + 0.10 + (grasp_z - bc[2] + 0.0)),
                                        downward, base_frame),
                                   tolerance=0.02, max_steps=250).ok
                open_gripper()
                result["placed"] = rel
                move_to_pose(Pose((tx, ty, lift_z), downward, base_frame),
                             tolerance=0.03, max_steps=250)
