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


bowl = locate(("black bowl", "bowl"))
plate = locate(("plate", "white plate", "round plate"))
if bowl is None or plate is None:
    result["error"] = "localization failed"
else:
    result["localized"] = True
    bc = bowl.geometry.pose.position
    zs = [p[2] for p in bowl.point_cloud.points]
    top_z = max(zs)
    pc = plate.geometry.pose.position
    plate_top = quantile((p[2] for p in plate.point_cloud.points), 0.9)

    # grasp the bowl rim: find the rim point on the side nearest the robot (min x)
    ext = bowl.geometry.extents
    radius = max(float(ext[0]), float(ext[1])) / 2.0
    gx = bc[0] - radius + 0.012
    gy = bc[1]
    gz = top_z - 0.02

    open_gripper()
    r1 = move_to_pose(Pose((gx, gy, top_z + 0.12), downward, base_frame), tolerance=0.02, max_steps=250)
    r2 = move_to_pose(Pose((gx, gy, gz), downward, base_frame), tolerance=0.015, max_steps=250)
    close_gripper()
    r3 = move_to_pose(Pose((gx, gy, top_z + 0.18), downward, base_frame), tolerance=0.02, max_steps=250)
    result["grasped"] = bool(r1.ok and r2.ok)

    # transport: keep the same rim offset relative to bowl center
    tx = pc[0] - radius + 0.012
    ty = pc[1]
    r4 = move_to_pose(Pose((tx, ty, top_z + 0.18), downward, base_frame), tolerance=0.02, max_steps=300)
    r5 = move_to_pose(Pose((tx, ty, plate_top + (top_z - float(min(zs))) + 0.04), downward, base_frame),
                      tolerance=0.02, max_steps=250)
    open_gripper()
    state = get_robot_state()
    for i in range(10):
        step(RobotAction(arms={"primary": ArmCommand(
            "joint_position",
            state.joint_positions["primary"],
            gripper_position=1.0,
        )}))
    move_to_pose(Pose((tx, ty, top_z + 0.18), downward, base_frame), tolerance=0.03, max_steps=200)
    result["placed"] = bool(r4.ok and r5.ok)
