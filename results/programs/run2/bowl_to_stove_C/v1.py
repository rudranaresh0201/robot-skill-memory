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


base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "grasped": False, "placed": False}

bowl = locate(("black bowl", "bowl"))
stove = locate(("flat stove", "stove", "black stove"))
if bowl is None or stove is None:
    result["error"] = "localization failed"
else:
    result["localized"] = True
    bpts = bowl.point_cloud.points
    bc = bowl.geometry.pose.position
    top_z = quantile((p[2] for p in bpts), 0.95)
    sc = stove.geometry.pose.position
    stove_z = quantile((p[2] for p in stove.point_cloud.points), 0.9)

    # Grasp the bowl rim: offset toward the robot-facing side of the rim
    xs = [p[0] for p in bpts]
    rim_x = quantile(xs, 0.1)
    grasp_z = top_z - 0.015
    open_gripper()
    r1 = move_to_pose(Pose((rim_x + 0.01, bc[1], top_z + 0.10), downward, base_frame),
                      tolerance=0.02, max_steps=250)
    r2 = move_to_pose(Pose((rim_x + 0.01, bc[1], grasp_z), downward, base_frame),
                      tolerance=0.015, max_steps=250)
    close_gripper()
    lift_z = top_z + 0.15
    r3 = move_to_pose(Pose((rim_x + 0.01, bc[1], lift_z), downward, base_frame),
                      tolerance=0.02, max_steps=250)
    # verify
    after = locate(("black bowl", "bowl"))
    if after is not None:
        az = quantile((p[2] for p in after.point_cloud.points), 0.5)
        result["grasped"] = az - quantile((p[2] for p in bpts), 0.5) > 0.02

    # Transport so that the bowl center lands over stove; rim offset preserved
    off_x = rim_x + 0.01 - bc[0]
    tx = sc[0] + off_x
    ty = sc[1]
    move_to_pose(Pose((tx, ty, lift_z + 0.05), downward, base_frame),
                 tolerance=0.02, max_steps=300)
    move_to_pose(Pose((tx, ty, stove_z + (top_z - quantile((p[2] for p in bpts), 0.0)) + 0.04),
                      downward, base_frame), tolerance=0.02, max_steps=250)
    open_gripper()
    state = get_robot_state()
    for i in range(15):
        step(RobotAction(arms={"primary": ArmCommand(
            "joint_position", state.joint_positions["primary"],
            gripper_position=1.0)}))
    move_to_pose(Pose((tx, ty, lift_z + 0.05), downward, base_frame),
                 tolerance=0.03, max_steps=200)
    result["placed"] = True
