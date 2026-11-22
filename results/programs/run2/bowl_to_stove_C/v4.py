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


def go(pose, tol=0.03, steps=120, tries=3):
    # Re-issue the move: a timed-out call usually still moved the arm part of the way.
    last = None
    for i in range(tries):
        last = move_to_pose(pose, tolerance=tol, max_steps=steps)
        if last.ok:
            return last
        if last.status == "adapter_failed":
            return last
    return last


base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "grasped": False, "placed": False}

bowl = locate(("bowl", "silver bowl", "black bowl"))
stove = locate(("flat stove", "stove", "black stove"))
if bowl is None or stove is None:
    result["error"] = "localization failed"
else:
    result["localized"] = True
    bpts = bowl.point_cloud.points
    bc = bowl.geometry.pose.position
    top_z = quantile((p[2] for p in bpts), 0.95)
    bottom_z = quantile((p[2] for p in bpts), 0.02)
    bowl_h = top_z - bottom_z
    base_med_z = quantile((p[2] for p in bpts), 0.5)
    sc = stove.geometry.pose.position
    stove_z = quantile((p[2] for p in stove.point_cloud.points), 0.9)

    rim_x = quantile((p[0] for p in bpts), 0.05)
    gx = rim_x + 0.015
    grasp_z = top_z - 0.025
    open_gripper()
    r0 = go(Pose((gx, bc[1], top_z + 0.15), downward, base_frame), 0.04, 150)
    result["hover_ok"] = bool(r0.ok)
    r1 = go(Pose((gx, bc[1], top_z + 0.07), downward, base_frame), 0.03, 100)
    r2 = go(Pose((gx, bc[1], grasp_z), downward, base_frame), 0.02, 100)
    result["approach_ok"] = bool(r1.ok and r2.ok)
    if r2.ok:
        close_gripper()
        lift_z = top_z + 0.12
        r3 = go(Pose((gx, bc[1], lift_z), downward, base_frame), 0.04, 120)
        after = locate(("bowl", "silver bowl", "black bowl"))
        if after is not None:
            az = quantile((p[2] for p in after.point_cloud.points), 0.5)
            result["grasped"] = (az - base_med_z) > 0.02

        off_x = gx - bc[0]
        tx = sc[0] + off_x
        ty = sc[1]
        r4 = go(Pose((tx, ty, lift_z + 0.03), downward, base_frame), 0.04, 150)
        release_z = stove_z + bowl_h + 0.04
        r5 = go(Pose((tx, ty, release_z), downward, base_frame), 0.03, 100)
        open_gripper()
        state = get_robot_state()
        for i in range(10):
            step(RobotAction(arms={"primary": ArmCommand(
                "joint_position", state.joint_positions["primary"],
                gripper_position=1.0,
                embodiment=state.embodiment)}))
        result["placed"] = bool(r4.ok and r5.ok)
