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
    dx, dy = tx - pc[0], ty - pc[1]
    dist = math.sqrt(dx * dx + dy * dy)
    result["localized"] = True
    result["plate"] = [float(v) for v in pc]
    result["target"] = [tx, ty]
    if dist < 0.02:
        result["pushed"] = True
    else:
        ux, uy = dx / dist, dy / dist
        off = radius + 0.05
        sx, sy = pc[0] - ux * off, pc[1] - uy * off
        push_z = 0.045
        hi = 0.30
        close_gripper()
        ok = go(sx, sy, hi)
        result["at_hi"] = ok
        if ok:
            ok = go(sx, sy, push_z + 0.08, 0.02, 250)
            ok = go(sx, sy, push_z, 0.02, 250) if ok else False
        result["at_start"] = ok
        if ok:
            good = True
            steps = 3
            for i in range(1, steps + 1):
                f = dist * i / steps
                if not go(sx + ux * f, sy + uy * f, push_z, 0.02, 200):
                    good = False
                    break
            result["pushed"] = good
            go(sx + ux * dist, sy + uy * dist, hi, 0.03, 250)
