import math


def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]


base_frame = get_robot_state().base_frame
down = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "grasped": False, "placed": False, "moves": []}


def locate(queries):
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None


def go(x, y, z, tol=0.03, steps=200):
    # try the pose, then retry once with looser tolerance and more steps
    for t, s in ((tol, steps), (tol * 1.7, steps + 100)):
        try:
            r = move_to_pose(Pose((x, y, z), down, base_frame), tolerance=t, max_steps=s)
            if r.ok:
                result["moves"].append(True)
                return True
        except Exception as e:
            result["moves"].append(str(e))
    result["moves"].append(False)
    return False


plate = locate(("white plate", "plate", "flat round plate"))
stove = locate(("flat stove", "stove", "black stove"))
if plate is None or stove is None:
    result["notes"] = "localization failed"
else:
    result["localized"] = True
    pc = plate.geometry.pose.position
    sc = stove.geometry.pose.position
    pts = plate.point_cloud.points
    top_z = quantile((p[2] for p in pts), 0.95)
    dx = sc[0] - pc[0]
    dy = sc[1] - pc[1]
    n = math.sqrt(dx * dx + dy * dy)
    if n < 1e-6:
        n = 1.0
    ux, uy = dx / n, dy / n
    push = max(0.05, n - 0.12)
    start_off = 0.14
    sx = pc[0] - ux * start_off
    sy = pc[1] - uy * start_off
    z_push = max(top_z + 0.015, 0.035)
    result["plate_center"] = tuple(pc)
    result["stove_center"] = tuple(sc)
    try:
        close_gripper()
        cur = get_robot_state().end_effector_poses["primary"].position
        # raise first to a safe height above current location
        go(cur[0], cur[1], max(cur[2], 0.25), 0.04, 150)
        ok1 = go(sx, sy, 0.25, 0.04, 250)
        ok1b = ok1 and go(sx, sy, z_push + 0.08, 0.03, 200)
        ok2 = ok1b and go(sx, sy, z_push, 0.025, 200)
        ok3 = True
        if ok2:
            for i in (1, 2, 3):
                f = push * i / 3.0
                ok3 = go(sx + ux * f, sy + uy * f, z_push, 0.03, 200) and ok3
            go(sx + ux * push, sy + uy * push, z_push + 0.12, 0.04, 150)
        result["placed"] = bool(ok2 and ok3)
    except Exception as e:
        result["notes"] = str(e)
