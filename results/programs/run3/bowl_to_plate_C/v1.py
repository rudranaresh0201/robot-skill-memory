def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]


base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "grasped": False, "placed": False}


def locate(queries):
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None


bowl = locate(("black bowl", "akita black bowl", "bowl"))
plate = locate(("plate", "white plate", "round plate"))
if bowl is None or plate is None:
    result["error"] = "localization failed"
else:
    result["localized"] = True
    bc = bowl.geometry.pose.position
    bzs = [p[2] for p in bowl.point_cloud.points]
    top_z = max(bzs)
    pc = plate.geometry.pose.position
    plate_top = quantile((p[2] for p in plate.point_cloud.points), 0.9)

    # grasp the bowl rim: find the rim point at the +x extent of the cloud
    xs = [p[0] for p in bowl.point_cloud.points]
    ext_x = (max(xs) - min(xs)) / 2.0
    gx = bc[0] + 0.0
    gy = bc[1] + ext_x - 0.01
    gz = top_z - 0.015

    open_gripper()
    r = move_to_pose(Pose((gx, gy, top_z + 0.10), downward, base_frame),
                     tolerance=0.02, max_steps=250)
    if r.ok:
        r = move_to_pose(Pose((gx, gy, gz), downward, base_frame),
                         tolerance=0.015, max_steps=250)
    if r.ok:
        close_gripper()
        result["grasped"] = True
        lift_z = top_z + 0.15
        r = move_to_pose(Pose((gx, gy, lift_z), downward, base_frame),
                         tolerance=0.02, max_steps=250)
        if r.ok:
            # keep same grasp offset relative to bowl center
            tx = pc[0] + (gx - bc[0])
            ty = pc[1] + (gy - bc[1])
            r = move_to_pose(Pose((tx, ty, lift_z), downward, base_frame),
                             tolerance=0.02, max_steps=300)
        if r.ok:
            height = top_z - min(bzs)
            r = move_to_pose(Pose((tx, ty, plate_top + height + 0.04), downward, base_frame),
                             tolerance=0.02, max_steps=250)
        if r.ok:
            open_gripper()
            result["placed"] = True
            move_to_pose(Pose((tx, ty, lift_z), downward, base_frame),
                         tolerance=0.03, max_steps=200)
        else:
            result["error"] = "motion failed: " + str(r.status)
    else:
        result["error"] = "approach failed"
