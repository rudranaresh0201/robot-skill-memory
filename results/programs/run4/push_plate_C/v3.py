import math

base_frame = get_robot_state().base_frame
down = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "pushed": False}


def near(r):
    return r.ok or (r.error is not None and r.error.details.get("joint_error_rad", 9) < 0.05)


def locate(queries):
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None


def go(x, y, z, tol=0.02, steps=300):
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
    radius = max(0.08, min(0.14, 0.5 * max(float(ext[0]), float(ext[1]))))
    tx, ty = sc[0] + 0.36, sc[1]
    result["localized"] = True
    result["plate"] = [float(v) for v in pc]
    result["target"] = [float(tx), float(ty)]
    hi = 0.30
    push_z = 0.06
    off = radius + 0.05
    close_gripper()

    def push_leg(x0, y0, ux, uy, dist):
        # start behind (x0,y0) opposite to push direction, push by dist
        sx, sy = x0 - ux * off, y0 - uy * off
        if not go(sx, sy, hi):
            return False
        if not go(sx, sy, push_z + 0.08, 0.02, 250):
            return False
        if not go(sx, sy, push_z, 0.02, 250):
            return False
        n = 3
        for i in range(1, n + 1):
            f = dist * i / n
            if not go(sx + ux * f, sy + uy * f, push_z, 0.02, 200):
                return False
        go(sx + ux * dist, sy + uy * dist, hi, 0.03, 250)
        return True

    dx = tx - pc[0]
    ok1 = True
    if dx > 0.02:
        ok1 = push_leg(pc[0], pc[1], 1.0, 0.0, dx)
    result["leg1"] = ok1
    ok2 = False
    if ok1:
        dy = ty - pc[1]
        if abs(dy) < 0.02:
            ok2 = True
        else:
            sgn = 1.0 if dy > 0 else -1.0
            ok2 = push_leg(tx, pc[1], 0.0, sgn, abs(dy))
    result["leg2"] = ok2
    result["pushed"] = ok1 and ok2
