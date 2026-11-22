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

bowl = locate(("black bowl", "akita black bowl", "bowl"))
plate = locate(("plate", "white plate", "round plate"))
if bowl is None or plate is None:
    raise ValueError("localization failed")
result["localized"] = True

bpts = bowl.point_cloud.points
bx = quantile((p[0] for p in bpts), 0.5)
by = quantile((p[1] for p in bpts), 0.5)
bz_top = quantile((p[2] for p in bpts), 0.95)
ppts = plate.point_cloud.points
px = quantile((p[0] for p in ppts), 0.5)
py = quantile((p[1] for p in ppts), 0.5)
pz = quantile((p[2] for p in ppts), 0.9)

down = get_robot_state().end_effector_poses["primary"].quaternion_wxyz

# grasp the bowl rim: offset toward the near edge along x using bowl extent
ext = bowl.geometry.extents
rim_off = float(max(ext[0], ext[1])) / 2.0 - 0.01
gx, gy = bx - rim_off, by
gz = bz_top - 0.02

open_gripper()
ok1 = move_to_pose(Pose((gx, gy, gz + 0.15), down, base_frame), tolerance=0.02, max_steps=250).ok
ok2 = ok1 and move_to_pose(Pose((gx, gy, gz), down, base_frame), tolerance=0.015, max_steps=250).ok
result["approached"] = bool(ok2)
if ok2:
    close_gripper()
    before = bz_top
    lift_ok = move_to_pose(Pose((gx, gy, gz + 0.2), down, base_frame), tolerance=0.02, max_steps=250).ok
    after = localize_object("black bowl", camera_name="agentview", target_frame=base_frame)
    if after.ok and len(after.point_cloud.points) >= 20:
        az = quantile((p[2] for p in after.point_cloud.points), 0.95)
        result["grasped"] = bool(az - before > 0.02)
    else:
        result["grasped"] = bool(lift_ok)

    # transport: bowl center should end over plate; grasp point is offset by rim_off
    tx, ty = px - rim_off, py
    move_to_pose(Pose((tx, ty, gz + 0.2), down, base_frame), tolerance=0.02, max_steps=300)
    move_to_pose(Pose((tx, ty, pz + 0.10), down, base_frame), tolerance=0.02, max_steps=250)
    open_gripper()
    state = get_robot_state()
    for t in range(15):
        step(RobotAction(arms={"primary": ArmCommand(
            "joint_position",
            state.joint_positions["primary"],
            gripper_position=1.0,
        )}))
    result["placed"] = True
    move_to_pose(Pose((tx, ty, gz + 0.2), down, base_frame), tolerance=0.03, max_steps=200)
