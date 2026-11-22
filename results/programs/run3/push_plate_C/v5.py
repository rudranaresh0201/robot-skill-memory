import math

base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "stove_localized": False, "pushed": False, "log": []}


def find(queries):
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None


def mv(x, y, z, tol=0.03, steps=200, tries=3):
    # Re-issue the move: a timeout with moderate error usually means slow, and repeats accumulate.
    r = None
    for i in range(tries):
        r = move_to_pose(Pose((x, y, z), downward, base_frame),
                         tolerance=tol, max_steps=steps)
        if r.ok:
            return True
    return False


def push(c, tx, ty, radius, push_z, hover_z, tag):
    dx = tx - c[0]
    dy = ty - c[1]
    dist = math.sqrt(dx * dx + dy * dy)
    if dist < 0.03:
        return True
    ux, uy = dx / dist, dy / dist
    sx = c[0] - ux * (radius + 0.04)
    sy = c[1] - uy * (radius + 0.04)
    ok1 = mv(sx, sy, hover_z, 0.04, 200, 3)
    result[tag + "_above"] = ok1
    if not ok1:
        return False
    ok2 = mv(sx, sy, push_z, 0.03, 150, 3)
    result[tag + "_desc"] = ok2
    if not ok2:
        mv(sx, sy, hover_z, 0.05, 150, 2)
        return False
    total = dist + 0.01 + radius * 0.0
    n = max(1, int(total / 0.10) + 1)
    good = True
    for i in range(1, n + 1):
        f = i / n
        px = sx + ux * (radius + 0.04 + total) * f
        py = sy + uy * (radius + 0.04 + total) * f
        if not mv(px, py, push_z, 0.04, 150, 2):
            good = False
            break
    result[tag + "_push"] = good
    mv(sx + ux * (radius + 0.04 + total), sy + uy * (radius + 0.04 + total),
       hover_z, 0.06, 150, 2)
    return good


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
        tx = s[0] + 0.32
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
