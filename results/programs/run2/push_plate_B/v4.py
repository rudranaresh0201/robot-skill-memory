import math

base_frame = get_robot_state().base_frame
result = {"localized": False, "approached": False, "pushed": False}

def locate(queries):
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None

def cur_pos():
    p = get_robot_state().end_effector_poses["primary"].position
    return (float(p[0]), float(p[1]), float(p[2]))

def goto(x, y, z, quat, seg=0.10, tol=0.03, steps=150):
    """Move in short Cartesian segments; return True if finally near target."""
    for it in range(12):
        c = cur_pos()
        dx, dy, dz = x - c[0], y - c[1], z - c[2]
        d = math.sqrt(dx * dx + dy * dy + dz * dz)
        if d < tol:
            return True
        k = 1.0 if d <= seg else seg / d
        pose = Pose((c[0] + dx * k, c[1] + dy * k, c[2] + dz * k), quat, base_frame)
        r = move_to_pose(pose, tolerance=tol, max_steps=steps)
        if not r.ok:
            r = move_to_pose(pose, tolerance=0.05, max_steps=steps)
            if not r.ok:
                ik = solve_ik(pose)
                if ik.ok:
                    move_to_joints(ik.joint_positions, tolerance=0.05, max_steps=steps)
    c = cur_pos()
    return math.sqrt((x - c[0]) ** 2 + (y - c[1]) ** 2 + (z - c[2]) ** 2) < 0.05

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
    radius = min(0.14, max(xs[-1] - xs[0], ys[-1] - ys[0]) / 2.0)
    sp = stove.point_cloud.points
    scx = sum(float(p[0]) for p in sp) / len(sp)
    sy = sum(float(p[1]) for p in sp) / len(sp)
    tx = scx + 0.33
    ty = sy
    dx, dy = tx - px, ty - py
    dist = max(0.02, math.sqrt(dx * dx + dy * dy))
    ux, uy = dx / dist, dy / dist
    down = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
    z = zmin + 0.025
    hover = zmin + 0.17
    sx0 = px - ux * (radius + 0.04)
    sy0 = py - uy * (radius + 0.04)
    close_gripper()
    c = cur_pos()
    goto(c[0], c[1], max(c[2], hover), down)
    ok1 = goto(sx0, sy0, hover, down)
    ok2 = goto(sx0, sy0, z, down, seg=0.08) if ok1 else False
    result["approached"] = bool(ok1 and ok2)
    if ok1 and ok2:
        n = max(1, int(dist / 0.05))
        okp = True
        for i in range(1, n + 1):
            f = i / n
            if not goto(sx0 + dx * f, sy0 + dy * f, z, down, seg=0.06, tol=0.03, steps=120):
                okp = False
        result["pushed"] = okp
        c = cur_pos()
        goto(c[0], c[1], hover, down, tol=0.05)
