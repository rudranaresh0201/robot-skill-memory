def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]

def locate(queries):
    base = get_robot_state().base_frame
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None

base_frame = get_robot_state().base_frame
result = {"localized": False, "grasped": False, "placed": False}

bowl_queries = ("silver bowl", "gray patterned bowl", "bowl")
plate = locate(("plate", "white plate", "round plate"))
bowl = locate(bowl_queries)
if bowl is None or plate is None:
    raise ValueError("localization failed")
result["localized"] = True

ppts = plate.point_cloud.points
px = quantile((p[0] for p in ppts), 0.5)
py = quantile((p[1] for p in ppts), 0.5)
pz = quantile((p[2] for p in ppts), 0.9)

down = get_robot_state().end_effector_poses["primary"].quaternion_wxyz

attempts = [(-1.0, 0.025), (1.0, 0.025), (-1.0, 0.015)]
held_side = None
held_gz = None
result["attempts"] = 0
for side, depth in attempts:
    if bowl is None:
        bowl = locate(bowl_queries)
        if bowl is None:
            break
    bpts = bowl.point_cloud.points
    xs = [p[0] for p in bpts]
    bx_lo = quantile(xs, 0.02)
    bx_hi = quantile(xs, 0.98)
    bx = 0.5 * (bx_lo + bx_hi)
    half = 0.5 * (bx_hi - bx_lo)
    by = quantile((p[1] for p in bpts), 0.5)
    bz_top = quantile((p[2] for p in bpts), 0.95)
    rim_off = max(half - 0.005, 0.02)
    gx, gy = bx + side * rim_off, by
    gz = bz_top - depth
    result["attempts"] += 1

    open_gripper()
    if not move_to_pose(Pose((gx, gy, gz + 0.15), down, base_frame), tolerance=0.02, max_steps=250).ok:
        bowl = None
        continue
    if not move_to_pose(Pose((gx, gy, gz), down, base_frame), tolerance=0.015, max_steps=250).ok:
        bowl = None
        continue
    result["approached"] = True
    close_gripper()
    move_to_pose(Pose((gx, gy, gz + 0.2), down, base_frame), tolerance=0.02, max_steps=250)
    after = locate(bowl_queries)
    if after is not None:
        az = quantile((p[2] for p in after.point_cloud.points), 0.95)
        if az - bz_top > 0.03:
            result["grasped"] = True
            held_side = side
            held_rim = rim_off
            held_gz = gz
            break
    # failed: release and retry
    open_gripper()
    move_to_pose(Pose((gx, gy, gz + 0.2), down, base_frame), tolerance=0.03, max_steps=200)
    bowl = locate(bowl_queries)

if result["grasped"]:
    tx = px + held_side * held_rim
    ty = py
    move_to_pose(Pose((tx, ty, held_gz + 0.2), down, base_frame), tolerance=0.02, max_steps=300)
    move_to_pose(Pose((tx, ty, pz + 0.10), down, base_frame), tolerance=0.02, max_steps=250)
    open_gripper()
    state = get_robot_state()
    for t in range(20):
        step(RobotAction(arms={"primary": ArmCommand(
            "joint_position",
            state.joint_positions["primary"],
            gripper_position=1.0,
            embodiment=state.embodiment,
        )}))
    result["placed"] = True
    move_to_pose(Pose((tx, ty, held_gz + 0.2), down, base_frame), tolerance=0.03, max_steps=200)
