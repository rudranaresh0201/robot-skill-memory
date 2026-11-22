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

def ee_dist(pose):
    p = get_robot_state().end_effector_poses["primary"].position
    return math.sqrt((p[0] - pose.position[0]) ** 2 + (p[1] - pose.position[1]) ** 2 + (p[2] - pose.position[2]) ** 2)

def go(pose, tol=0.03, steps=300, tries=2, accept=0.05):
    r = None
    for i in range(tries):
        r = move_to_pose(pose, tolerance=tol, max_steps=steps)
        if r.ok:
            return True
        if ee_dist(pose) < accept:
            return True
    return False

result = {"localized": False, "grasped": False, "placed": False}
base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz

bowl = locate(("silver bowl", "black bowl", "bowl"))
stove = locate(("flat stove", "black stove", "stove", "cooktop"))
if bowl is None or stove is None:
    result["error"] = "localization failed"
else:
    result["localized"] = True
    pts = bowl.point_cloud.points
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    bc = (quantile(xs, 0.5), quantile(ys, 0.5), quantile(zs, 0.5))
    top_z = quantile(zs, 0.95)
    bottom_z = min(zs)
    spts = stove.point_cloud.points
    sc = (quantile([p[0] for p in spts], 0.5), quantile([p[1] for p in spts], 0.5))
    stove_z = quantile([p[2] for p in spts], 0.9)
    result["stove_xy"] = sc

    grasp_z = top_z - 0.02
    cands = [
        (min(xs) + 0.01, bc[1]),
        (max(xs) - 0.01, bc[1]),
        (bc[0], min(ys) + 0.01),
        (bc[0], max(ys) - 0.01),
    ]
    open_gripper()
    chosen = None
    for (rx, ry) in cands:
        pre = move_to_pose(Pose((rx, ry, grasp_z + 0.15), downward, base_frame), tolerance=0.03, max_steps=150)
        if not pre.ok:
            continue
        d = move_to_pose(Pose((rx, ry, grasp_z), downward, base_frame), tolerance=0.02, max_steps=150)
        if d.ok:
            chosen = (rx, ry)
            break
    result["approached"] = chosen is not None
    if chosen is not None:
        rx, ry = chosen
        close_gripper()
        lift_z = grasp_z + 0.13
        l = go(Pose((rx, ry, lift_z), downward, base_frame), 0.03, 200, 2)
        result["lifted"] = l
        if l:
            result["grasped"] = True
            tx = sc[0] + (rx - bc[0])
            ty = sc[1] + (ry - bc[1])
            ok = True
            nseg = 5
            for k in range(1, nseg + 1):
                frac = k / nseg
                wx = rx + (tx - rx) * frac
                wy = ry + (ty - ry) * frac
                if not go(Pose((wx, wy, lift_z), downward, base_frame), 0.05, 300, 2, 0.06):
                    ok = False
                    break
            result["transported"] = ok
            if ok:
                rel_z = stove_z + (grasp_z - bottom_z) + 0.04
                d = go(Pose((tx, ty, rel_z), downward, base_frame), 0.03, 250, 2, 0.04)
                result["descended"] = d
                open_gripper()
                state = get_robot_state()
                for i in range(15):
                    step(RobotAction(arms={"primary": ArmCommand(
                        "joint_position",
                        state.joint_positions["primary"],
                        gripper_position=1.0,
                        embodiment=state.embodiment)}))
                result["placed"] = d
                go(Pose((tx, ty, lift_z), downward, base_frame), 0.05, 150, 1, 0.08)
