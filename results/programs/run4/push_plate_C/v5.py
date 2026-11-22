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
    radius = max(0.08, min(0.12, 0.5 * max(float(ext[0]), float(ext[1]))))
    px, py = float(pc[0]), float(pc[1])
    # the stove front is ~0.15 m further in x than the plate and at the stove's y
    tx, ty = px + 0.15, float(sc[1])
    result["localized"] = True
    result["plate"] = [px, py, float(pc[2])]
    result["target"] = [tx, ty]
    hi = 0.30
    push_z = 0.07
    off = radius + 0.025
    close_gripper()

    def push_leg(x0, y0, ux, uy, dist, n):
        sx, sy = x0 - ux * off, y0 - uy * off
        if not go(sx, sy, hi):
            return False
        if not go(sx, sy, push_z + 0.08, 0.02, 250):
            return False
        if not go(sx, sy, push_z, 0.02, 250):
            return False
        for i in range(1, n + 1):
            f = dist * i / n
            if not go(sx + ux * f, sy + uy * f, push_z, 0.03, 200):
                return False
        go(sx + ux * dist, sy + uy * dist, hi, 0.03, 250)
        return True

    # Leg 1: sideways (+y) push, away from the bowl/bottle that sit in front (+x) of the plate
    dy = ty - py
    ok1 = True
    if abs(dy) > 0.02:
        sgn = 1.0 if dy > 0 else -1.0
        ok1 = push_leg(px, py, 0.0, sgn, abs(dy), 4)
    result["leg1"] = ok1

    # Leg 2: push forward (+x) to the stove front, now clear of the bowl
    ok2 = False
    if ok1:
        dx = tx - px
        if dx > 0.02:
            ok2 = push_leg(px, ty, 1.0, 0.0, dx, 3)
        else:
            ok2 = True
    result["leg2"] = ok2
    result["pushed"] = ok1 and ok2
