import math

base_frame = get_robot_state().base_frame
result = {"localized": False, "pushed": False}

def locate(queries):
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None

def goto(x, y, z, quat, tol=0.03, steps=300):
    pose = Pose((x, y, z), quat, base_frame)
    for t, s in ((tol, steps), (tol * 1.7, steps), (0.06, steps)):
        r = move_to_pose(pose, tolerance=t, max_steps=s)
        if r.ok:
            return True
    ik = solve_ik(pose)
    if ik.ok:
        r = move_to_joints(ik.joint_positions, tolerance=0.03, max_steps=300)
        if r.ok:
            return True
        r = move_to_joints(ik.joint_positions, tolerance=0.05, max_steps=300)
        return bool(r.ok)
    return False

plate = locate(("plate", "white plate", "round plate", "flat plate"))
stove = locate(("flat stove", "stove", "black stove"))
if plate is None or stove is None:
    result["error"] = "localization failed"
else:
    result["localized"] = True
    pp = plate.point_cloud.points
    xs = sorted(float(p[0]) for p in pp)
    ys = sorted(float(p[1]) for p in pp)
    zs = sorted(float(p[2]) for p in pp)
    n0 = len(pp)
    px = xs[n0 // 2]
    py = ys[n0 // 2]
    zmin = zs[n0 // 20]
    radius = min(0.17, max(xs[-1] - xs[0], ys[-1] - ys[0]) / 2.0)
    sp = stove.point_cloud.points
    sy = sum(float(p[1]) for p in sp) / len(sp)
    sxmax = max(float(p[0]) for p in sp)
    tx = sxmax + 0.08
    ty = sy
    dx, dy = tx - px, ty - py
    dist = math.sqrt(dx * dx + dy * dy)
    if dist < 0.02:
        dist = 0.02
    ux, uy = dx / dist, dy / dist
    down = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
    z = zmin + 0.04
    hover = z + 0.15
    sx0 = px - ux * (radius + 0.05)
    sy0 = py - uy * (radius + 0.05)
    close_gripper()
    cur = get_robot_state().end_effector_poses["primary"].position
    # lift first to a safe height
    goto(float(cur[0]), float(cur[1]), max(float(cur[2]), hover), down, 0.04, 200)
    ok1 = goto(sx0, sy0, hover, down)
    ok2 = False
    if ok1:
        ok2 = goto(sx0, sy0, z, down, 0.03, 300)
    result["approached"] = bool(ok1 and ok2)
    if ok1 and ok2:
        n = max(1, int(dist / 0.05))
        okp = True
        for i in range(1, n + 1):
            f = i / n
            r = move_to_pose(
                Pose((sx0 + dx * f, sy0 + dy * f, z), down, base_frame),
                tolerance=0.03, max_steps=200)
            if not r.ok:
                okp = False
        result["pushed"] = okp
        cur = get_robot_state().end_effector_poses["primary"].position
        move_to_pose(Pose((float(cur[0]), float(cur[1]), hover), down, base_frame),
                     tolerance=0.05, max_steps=200)
