import math

base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "stove_localized": False, "pushed": False}


def find(queries):
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None


def mv(x, y, z, tol, steps):
    return move_to_pose(Pose((x, y, z), downward, base_frame),
                        tolerance=tol, max_steps=steps)


def push(c, tx, ty, radius, push_z, hover_z, tag):
    dx = tx - c[0]
    dy = ty - c[1]
    dist = math.sqrt(dx * dx + dy * dy)
    if dist < 0.03:
        return True
    ux, uy = dx / dist, dy / dist
    sx = c[0] - ux * (radius + 0.04)
    sy = c[1] - uy * (radius + 0.04)
    ex = c[0] + ux * (dist + 0.01)
    ey = c[1] + uy * (dist + 0.01)
    r1 = mv(sx, sy, hover_z, 0.08, 500)
    result[tag + "_above"] = r1.ok
    r2 = mv(sx, sy, push_z, 0.06, 400)
    result[tag + "_desc"] = r2.ok
    if not r2.ok:
        mv(sx, sy, hover_z, 0.1, 300)
        return False
    r3 = mv(ex, ey, push_z, 0.08, 500)
    result[tag + "_push"] = r3.ok
    mv(ex, ey, hover_z, 0.1, 300)
    return r3.ok


names = ("plate", "white plate", "round plate")
plate = find(names)
stove = find(("black stove", "flat stove", "stove burner"))

if plate is None:
    result["error"] = "plate not found"
else:
    result["localized"] = True
    c = plate.geometry.pose.position
    ext = plate.geometry.extents
    radius = max(0.07, min(0.12, max(ext[0], ext[1]) / 2.0))
    low_z = min(float(p[2]) for p in plate.point_cloud.points)
    push_z = low_z + 0.035
    hover_z = low_z + 0.15

    if stove is not None:
        result["stove_localized"] = True
        s = stove.geometry.pose.position
        tx = s[0] + 0.34
        ty = s[1]
    else:
        tx = c[0] + 0.15
        ty = c[1] + 0.26
    result["target"] = [tx, ty]
    close_gripper()
    ok = push(c, tx, ty, radius, push_z, hover_z, "p1")
    result["pushed"] = ok
    again = find(names)
    if again is not None:
        c2 = again.geometry.pose.position
        remaining = math.sqrt((tx - c2[0]) ** 2 + (ty - c2[1]) ** 2)
        result["remaining"] = remaining
        if remaining > 0.06:
            ok2 = push(c2, tx, ty, radius, push_z, hover_z, "p2")
            result["pushed"] = ok2
