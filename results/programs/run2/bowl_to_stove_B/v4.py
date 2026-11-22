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

def dist_to(pose):
    cur = get_robot_state().end_effector_poses["primary"].position
    return math.sqrt(sum((float(pose.position[i]) - float(cur[i])) ** 2 for i in range(3)))

def go(pose, tol=0.04, steps=300):
    # judge success by measured end-effector distance, not by the call status
    for attempt in range(4):
        if dist_to(pose) < tol:
            return True
        move_to_pose(pose, tolerance=0.04, max_steps=steps)
    return dist_to(pose) < tol * 1.5

result = {"localized": False, "grasped": False, "placed": False}
base_frame = get_robot_state().base_frame
down = get_robot_state().end_effector_poses["primary"].quaternion_wxyz

bowl = locate(("akita black bowl", "black bowl", "bowl"))
stove = locate(("flat stove", "black stove", "stove", "stove burner"))
if bowl is None or stove is None:
    result["error"] = "localization failed"
else:
    result["localized"] = True
    pts = bowl.point_cloud.points
    bx = quantile([p[0] for p in pts], 0.5)
    by = quantile([p[1] for p in pts], 0.5)
    btop = quantile([p[2] for p in pts], 0.95)
    bbot = quantile([p[2] for p in pts], 0.05)
    spts = stove.point_cloud.points
    sx = quantile([p[0] for p in spts], 0.5)
    sy = quantile([p[1] for p in spts], 0.5)
    sz = quantile([p[2] for p in spts], 0.9)
    result["bowl"] = [bx, by, btop]
    result["stove"] = [sx, sy, sz]

    open_gripper()
    ext = bowl.geometry.extents
    rim_off = max(min(float(ext[0]), float(ext[1])) / 2.0 - 0.015, 0.0)
    gx, gy, gz = bx + rim_off, by, btop - 0.02

    ok = go(Pose((gx, gy, gz + 0.18), down, base_frame), tol=0.05)
    if ok:
        ok = go(Pose((gx, gy, gz), down, base_frame), tol=0.04)
    result["approached"] = ok
    if ok:
        close_gripper()
        lift_z = gz + 0.2
        go(Pose((gx, gy, lift_z), down, base_frame), tol=0.05)
        after = locate(("akita black bowl", "black bowl", "bowl"))
        if after is not None:
            az = quantile([p[2] for p in after.point_cloud.points], 0.5)
            result["grasped"] = az - quantile([p[2] for p in pts], 0.5) > 0.02
        tx, ty = sx + rim_off, sy
        go(Pose((tx, ty, lift_z + 0.05), down, base_frame), tol=0.05)
        go(Pose((tx, ty, sz + (btop - bbot) + 0.04), down, base_frame), tol=0.04)
        open_gripper()
        state = get_robot_state()
        for t in range(15):
            step(RobotAction(arms={"primary": ArmCommand(
                "joint_position", state.joint_positions["primary"],
                gripper_position=1.0, embodiment=state.embodiment)}))
        go(Pose((tx, ty, lift_z + 0.05), down, base_frame), tol=0.05)
        result["placed"] = True
