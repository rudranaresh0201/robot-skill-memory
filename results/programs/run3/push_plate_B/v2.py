import math

def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]

def center_of(points):
    return (quantile((p[0] for p in points), 0.5),
            quantile((p[1] for p in points), 0.5),
            quantile((p[2] for p in points), 0.5))

def locate(queries):
    base = get_robot_state().base_frame
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None

result = {"localized": False, "pushed": False}
base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz

plate = locate(("white plate", "plate", "flat round plate"))
stove = locate(("black flat stove", "stove", "cooktop"))
if plate is None or stove is None:
    result["error"] = "localization failed"
else:
    result["localized"] = True
    pc = center_of(plate.point_cloud.points)
    sc = center_of(stove.point_cloud.points)
    plate_zs = [float(p[2]) for p in plate.point_cloud.points]
    push_z = min(plate_zs) + 0.035
    hover_z = push_z + 0.12
    tx, ty = sc[0] + 0.34, sc[1]
    result["plate_center"] = list(pc)
    result["target"] = [tx, ty]

    standoff = 0.14
    close_gripper()

    def go(x, y, z, tol=0.02, steps=200):
        return move_to_pose(Pose((x, y, z), downward, base_frame),
                            tolerance=tol, max_steps=steps)

    def push_leg(sx, sy, ex, ey, n):
        # sx,sy: arm start (behind plate); ex,ey: arm end
        r = go(sx, sy, hover_z, 0.03, 250)
        if not r.ok:
            return False
        r = go(sx, sy, push_z, 0.02, 200)
        if not r.ok:
            return False
        for i in range(1, n + 1):
            x = sx + (ex - sx) * i / n
            y = sy + (ey - sy) * i / n
            r = go(x, y, push_z, 0.02, 150)
            if not r.ok:
                return False
        return True

    # Leg 1: push along +y (image-right) until plate y matches target y
    ok = True
    if abs(ty - pc[1]) > 0.03:
        sgn = 1.0 if ty > pc[1] else -1.0
        sx, sy = pc[0], pc[1] - sgn * standoff
        ex, ey = pc[0], ty - sgn * standoff
        ok = push_leg(sx, sy, ex, ey, 4)
        result["leg1"] = ok
        if ok:
            go(ex, ey, hover_z, 0.03, 150)
    # Leg 2: push along +x toward the front of the stove
    if ok:
        sgn = 1.0 if tx > pc[0] else -1.0
        sx, sy = pc[0] - sgn * standoff, ty
        ex, ey = tx - sgn * standoff, ty
        ok = push_leg(sx, sy, ex, ey, 4)
        result["leg2"] = ok
        if ok:
            go(ex, ey, hover_z, 0.03, 150)
    result["pushed"] = ok
