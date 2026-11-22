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

def go(pose, tol=0.03, steps=400):
    r = move_to_pose(pose, tolerance=tol, max_steps=steps)
    if not r.ok:
        r = move_to_pose(pose, tolerance=tol * 1.5, max_steps=steps)
    return r.ok

result = {"localized": False, "grasped": False, "placed": False}
base_frame = get_robot_state().base_frame
ee = get_robot_state().end_effector_poses["primary"]
down = ee.quaternion_wxyz

# move arm aside so it does not occlude the bowl
obs_ok = go(Pose((ee.position[0], ee.position[1] + 0.3, ee.position[2] + 0.1), down, base_frame))
result["observe_move"] = obs_ok

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

    open_gripper()
    ext = bowl.geometry.extents
    rim_off = max(min(float(ext[0]), float(ext[1])) / 2.0 - 0.015, 0.0)
    gx, gy, gz = bx + rim_off, by, btop - 0.02

    ok = go(Pose((gx, gy, gz + 0.15), down, base_frame))
    if ok:
        ok = go(Pose((gx, gy, gz), down, base_frame), tol=0.02)
    result["approached"] = ok
    if ok:
        close_gripper()
        lift_z = gz + 0.2
        go(Pose((gx, gy, lift_z), down, base_frame))
        after = locate(("akita black bowl", "black bowl", "bowl"))
        if after is not None:
            az = quantile([p[2] for p in after.point_cloud.points], 0.5)
            result["grasped"] = az - quantile([p[2] for p in pts], 0.5) > 0.02
        tx, ty = sx + rim_off, sy
        go(Pose((tx, ty, lift_z + 0.05), down, base_frame))
        go(Pose((tx, ty, sz + (btop - bbot) + 0.04), down, base_frame))
        open_gripper()
        state = get_robot_state()
        for t in range(15):
            step(RobotAction(arms={"primary": ArmCommand(
                "joint_position", state.joint_positions["primary"],
                gripper_position=1.0, embodiment=state.embodiment)}))
        go(Pose((tx, ty, lift_z + 0.05), down, base_frame))
        result["placed"] = True
