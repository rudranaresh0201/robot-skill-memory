import math

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

bowl = locate(("black bowl", "akita black bowl", "bowl"))
stove = locate(("flat stove", "black stove", "stove", "stove burner"))
if bowl is None or stove is None:
    result["error"] = "localization failed"
else:
    result["localized"] = True
    pts = bowl.point_cloud.points
    bx = quantile([p[0] for p in pts], 0.5)
    by = quantile([p[1] for p in pts], 0.5)
    btop = quantile([p[2] for p in pts], 0.95)
    spts = stove.point_cloud.points
    sx = quantile([p[0] for p in spts], 0.5)
    sy = quantile([p[1] for p in spts], 0.5)
    sz = quantile([p[2] for p in spts], 0.9)

    down = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
    open_gripper()
    # grasp bowl at rim: offset toward the edge using extents estimate
    ext = bowl.geometry.extents
    rim_off = max(float(ext[0]), float(ext[1])) / 2.0 - 0.01
    gx, gy, gz = bx + rim_off, by, btop - 0.015

    ok = move_to_pose(Pose((gx, gy, gz + 0.12), down, base_frame), tolerance=0.02, max_steps=250).ok
    if ok:
        ok = move_to_pose(Pose((gx, gy, gz), down, base_frame), tolerance=0.015, max_steps=250).ok
    result["approached"] = ok
    if ok:
        close_gripper()
        lift_z = gz + 0.2
        move_to_pose(Pose((gx, gy, lift_z), down, base_frame), tolerance=0.02, max_steps=250)
        after = locate(("black bowl", "akita black bowl", "bowl"))
        if after is not None:
            az = quantile([p[2] for p in after.point_cloud.points], 0.5)
            result["grasped"] = az - quantile([p[2] for p in pts], 0.5) > 0.02
        else:
            result["grasped"] = True
        # transport: keep same grasp offset relative to bowl center
        tx, ty = sx + rim_off, sy
        move_to_pose(Pose((tx, ty, lift_z), down, base_frame), tolerance=0.02, max_steps=300)
        rel = Pose((tx, ty, sz + (btop - quantile([p[2] for p in pts], 0.05)) + 0.05), down, base_frame)
        move_to_pose(rel, tolerance=0.02, max_steps=250)
        open_gripper()
        state = get_robot_state()
        for t in range(15):
            step(RobotAction(arms={"primary": ArmCommand(
                "joint_position", state.joint_positions["primary"],
                gripper_position=1.0, embodiment=state.embodiment)}))
        move_to_pose(Pose((tx, ty, lift_z), down, base_frame), tolerance=0.03, max_steps=200)
        result["placed"] = True
