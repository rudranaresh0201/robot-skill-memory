def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]

def locate(queries, camera_name="agentview"):
    bf = get_robot_state().base_frame
    for q in queries:
        f = localize_object(q, camera_name=camera_name, target_frame=bf)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None

def go(pose, tol=0.03, steps=300, retry=True):
    r = move_to_pose(pose, tolerance=tol, max_steps=steps)
    if r.ok:
        return True
    if not retry:
        return False
    r = move_to_pose(pose, tolerance=tol, max_steps=steps)
    return r.ok

def go_incremental(target, quat, frame, seg=0.08, reach=0.04, max_iter=12):
    stalls = 0
    for i in range(max_iter):
        cur = get_robot_state().end_effector_poses["primary"].position
        dx = target[0] - cur[0]
        dy = target[1] - cur[1]
        dz = target[2] - cur[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist < reach:
            return True
        f = min(seg, dist) / dist
        nxt = (cur[0] + dx * f, cur[1] + dy * f, cur[2] + dz * f)
        move_to_pose(Pose(nxt, quat, frame), tolerance=0.05, max_steps=100)
        new = get_robot_state().end_effector_poses["primary"].position
        moved = math.sqrt((new[0] - cur[0]) ** 2 + (new[1] - cur[1]) ** 2 + (new[2] - cur[2]) ** 2)
        if moved < 0.01:
            stalls += 1
            if stalls >= 2:
                return False
        else:
            stalls = 0
    cur = get_robot_state().end_effector_poses["primary"].position
    return math.sqrt(sum((target[k] - cur[k]) ** 2 for k in range(3))) < reach * 1.5

result = {"localized": False, "grasped": False, "placed": False}
base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz

bowl = locate(("black bowl", "akita black bowl", "bowl"))
plate = locate(("plate", "white plate", "round plate"))
if bowl is None or plate is None:
    result["error"] = "localization failed"
else:
    result["localized"] = True
    bpts = bowl.point_cloud.points
    bcx = quantile([p[0] for p in bpts], 0.5)
    bcy = quantile([p[1] for p in bpts], 0.5)
    btop = quantile([p[2] for p in bpts], 0.95)
    ppts = plate.point_cloud.points
    pc = (quantile([p[0] for p in ppts], 0.5), quantile([p[1] for p in ppts], 0.5))
    ptop = quantile([p[2] for p in ppts], 0.9)

    rim_y = quantile([p[1] for p in bpts], 0.05) + 0.008
    grasp_z = btop - 0.02
    open_gripper()
    ok = go(Pose((bcx, rim_y, grasp_z + 0.12), downward, base_frame), 0.03, 300)
    if ok:
        ok = go(Pose((bcx, rim_y, grasp_z), downward, base_frame), 0.03, 300)
    result["approach"] = ok
    if ok:
        close_gripper()
        lift_z = grasp_z + 0.18
        r = go(Pose((bcx, rim_y, lift_z), downward, base_frame), 0.04, 300)
        result["lifted"] = r
        if r:
            result["grasped"] = True
            off_y = rim_y - bcy
            tx, ty = pc[0], pc[1] + off_y
            r2 = go_incremental((tx, ty, lift_z), downward, base_frame)
            result["transported"] = r2
            if r2:
                rel_z = ptop + 0.10
                if rel_z < lift_z - 0.02:
                    result["descended"] = go_incremental((tx, ty, rel_z), downward, base_frame, seg=0.06, reach=0.03, max_iter=4)
                open_gripper()
                result["placed"] = True
            else:
                open_gripper()
            state = get_robot_state()
            for t in range(25):
                step(RobotAction(arms={"primary": ArmCommand(
                    "joint_position",
                    state.joint_positions["primary"],
                    gripper_position=1.0,
                    embodiment=state.embodiment,
                )}))
