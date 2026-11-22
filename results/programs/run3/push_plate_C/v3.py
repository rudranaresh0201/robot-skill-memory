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


def push(c, tx, ty, radius, push_z, hover_z, tag):
    """Push from plate center c toward (tx,ty). Returns True if completed."""
    dx = tx - c[0]
    dy = ty - c[1]
    dist = math.sqrt(dx * dx + dy * dy)
    if dist < 0.02:
        return True
    ux, uy = dx / dist, dy / dist
    sx = c[0] - ux * (radius + 0.05)
    sy = c[1] - uy * (radius + 0.05)
    ex = c[0] + ux * (dist + 0.02) - ux * radius * 0.0
    ey = c[1] + uy * (dist + 0.02)
    r1 = move_to_pose(Pose((sx, sy, hover_z), downward, base_frame),
                      tolerance=0.04, max_steps=400)
    result[tag + "_above"] = r1.ok
    if not r1.ok:
        return False
    r2 = move_to_pose(Pose((sx, sy, push_z), downward, base_frame),
                      tolerance=0.03, max_steps=300)
    result[tag + "_desc"] = r2.ok
    if not r2.ok:
        move_to_pose(Pose((sx, sy, hover_z), downward, base_frame),
                     tolerance=0.05, max_steps=200)
        return False
    good = True
    for f in (0.35, 0.7, 1.0):
        mx = sx + (ex - sx) * f
        my = sy + (ey - sy) * f
        r = move_to_pose(Pose((mx, my, push_z), downward, base_frame),
                         tolerance=0.04, max_steps=250)
        if not r.ok:
            good = False
            result[tag + "_fail_at"] = f
            break
    move_to_pose(Pose((ex, ey, hover_z), downward, base_frame),
                 tolerance=0.05, max_steps=200)
    return good


plate = find(("plate", "white plate", "round plate"))
stove = find(("black stove", "flat stove", "stove burner"))

if plate is None:
    result["error"] = "plate not found"
else:
    result["localized"] = True
    c = plate.geometry.pose.position
    ext = plate.geometry.extents
    radius = max(0.07, min(0.12, max(ext[0], ext[1]) / 2.0))
    low_z = min(float(p[2]) for p in plate.point_cloud.points)
    push_z = low_z + 0.03
    hover_z = low_z + 0.20

    if stove is not None:
        result["stove_localized"] = True
        s = stove.geometry.pose.position
        tx = s[0] + 0.33
        ty = s[1]
    else:
        tx = c[0] - 0.10
        ty = c[1] + 0.23
    close_gripper()
    ok = push(c, tx, ty, radius, push_z, hover_z, "direct")
    result["pushed"] = ok
    if not ok:
        # fallback: two legs, first along +x (from robot side), then along +y
        again = find(("plate", "white plate", "round plate"))
        c2 = again.geometry.pose.position if again is not None else c
        ok1 = push(c2, tx, c2[1], radius, push_z, hover_z, "legx")
        result["leg1"] = ok1
        again = find(("plate", "white plate", "round plate"))
        c3 = again.geometry.pose.position if again is not None else (tx, c2[1], c2[2])
        ok2 = push(c3, tx, ty, radius, push_z, hover_z, "legy")
        result["leg2"] = ok2
        result["pushed"] = ok1 and ok2
