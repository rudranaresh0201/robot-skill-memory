import math

base_frame = get_robot_state().base_frame
result = {"localized": False, "pushed": False}

def locate(queries):
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None

plate = locate(("plate", "white plate", "round plate"))
stove = locate(("flat stove", "stove", "black stove"))
if plate is None or stove is None:
    result["error"] = "localization failed"
else:
    result["localized"] = True
    pp = plate.point_cloud.points
    px = sum(float(p[0]) for p in pp) / len(pp)
    py = sum(float(p[1]) for p in pp) / len(pp)
    zmin = min(float(p[2]) for p in pp)
    xs = [float(p[0]) for p in pp]
    ys = [float(p[1]) for p in pp]
    radius = max(max(xs) - min(xs), max(ys) - min(ys)) / 2.0
    sp = stove.point_cloud.points
    sx = sum(float(p[0]) for p in sp) / len(sp)
    sy = sum(float(p[1]) for p in sp) / len(sp)
    sxmax = max(float(p[0]) for p in sp)
    tx = min(sxmax + 0.10, sx + 0.37)
    ty = sy
    dx, dy = tx - px, ty - py
    dist = math.sqrt(dx * dx + dy * dy)
    ux, uy = dx / dist, dy / dist
    down = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
    z = zmin + 0.035
    hover = z + 0.12
    sx0 = px - ux * (radius + 0.05)
    sy0 = py - uy * (radius + 0.05)
    close_gripper()
    ok1 = move_to_pose(Pose((sx0, sy0, hover), down, base_frame), tolerance=0.02, max_steps=250).ok
    ok2 = move_to_pose(Pose((sx0, sy0, z), down, base_frame), tolerance=0.015, max_steps=250).ok
    result["approached"] = bool(ok1 and ok2)
    if ok1 and ok2:
        travel = dist
        n = max(1, int(travel / 0.05))
        okp = True
        for i in range(1, n + 1):
            f = i / n
            r = move_to_pose(
                Pose((sx0 + dx * f, sy0 + dy * f, z), down, base_frame),
                tolerance=0.02, max_steps=150)
            if not r.ok:
                okp = False
                break
        result["pushed"] = okp
        cur = get_robot_state().end_effector_poses["primary"].position
        move_to_pose(Pose((float(cur[0]), float(cur[1]), hover), down, base_frame),
                     tolerance=0.03, max_steps=150)
