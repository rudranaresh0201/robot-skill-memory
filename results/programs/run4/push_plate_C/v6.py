import math

base_frame = get_robot_state().base_frame
down = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "pushed": False}


def near(r):
    return r.ok or (r.error is not None and r.error.details.get("joint_error_rad", 9) < 0.1)


def locate(queries):
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None


def go(x, y, z, tol=0.02, steps=500):
    r = move_to_pose(Pose((x, y, z), down, base_frame), tolerance=tol, max_steps=steps)
    return near(r)


plate = locate(("white plate", "plate", "round plate"))
stove = locate(("flat stove", "black stove", "stove"))
if plate is None or stove is None:
    result["error"] = "localization failed"
else:
    pc = plate.geometry.pose.position
    sc = stove.geometry.pose.position
    ext = plate.geometry.extents
    radius = max(0.08, min(0.11, 0.5 * max(float(ext[0]), float(ext[1]))))
    px, py = float(pc[0]), float(pc[1])
    tx, ty = px + 0.15, float(sc[1])
    result["localized"] = True
    result["plate"] = [px, py, float(pc[2])]
    result["target"] = [tx, ty]
    hi = 0.28
    push_z = 0.07
    off = radius + 0.02
    close_gripper()

    # rise straight up first
    cur = get_robot_state().end_effector_poses["primary"].position
    cx, cy = float(cur[0]), float(cur[1])
    result["rise"] = go(cx, cy, hi)

    def push(ux, uy, dist, n):
        sx, sy = px - ux * off, py - uy * off
        steps_log = []
        if not go(sx, sy, hi):
            return False
        if not go(sx, sy, push_z + 0.08, 0.02, 300):
            return False
        if not go(sx, sy, push_z, 0.02, 300):
            return False
        for i in range(1, n + 1):
            f = dist * i / n
            if not go(sx + ux * f, sy + uy * f, push_z, 0.03, 300):
                return False
        go(sx + ux * dist, sy + uy * dist, hi, 0.03, 300)
        return True

    dx, dy = tx - px, ty - py
    d = math.sqrt(dx * dx + dy * dy)
    ok = False
    if d > 0.02:
        ok = push(dx / d, dy / d, d, 6)
    result["diag"] = ok
    if not ok:
        # fallback: plain sideways push
        ok = push(0.0, 1.0 if dy > 0 else -1.0, abs(dy), 5)
        result["fallback"] = ok
    result["pushed"] = ok
