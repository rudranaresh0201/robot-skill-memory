import math

result = {"localized": False}

def near(r):
    return r.ok or (r.error is not None and r.error.details.get("joint_error_rad", 9) < 0.05)

Q = [0.0, 1.0, 0.0, 0.0]
def pose(x, y, z):
    return Pose([x, y, z], Q, "robot_base")

pl = localize_object("plate")
result["localized"] = bool(pl.ok)
if pl.ok:
    g = pl.geometry
    try:
        c = g.pose.position
    except Exception:
        c = g.center
    px, py, pz = float(c[0]), float(c[1]), float(c[2])
    result["plate"] = [px, py, pz]
    close_gripper()
    # push direction: toward the stove (smaller x, larger y in robot frame)
    dx, dy = -0.10, 0.23
    n = math.sqrt(dx * dx + dy * dy)
    ux, uy = dx / n, dy / n
    sx = px - ux * 0.13
    sy = py - uy * 0.13
    zlow = 0.045
    zhi = 0.32
    r = move_to_pose(pose(sx, sy, zhi))
    result["above"] = near(r)
    if near(r):
        r = move_to_pose(pose(sx, sy, zlow))
        result["descend"] = near(r)
        if near(r):
            ok = True
            for f in (0.25, 0.5, 0.75, 1.0):
                tx = sx + dx * f
                ty = sy + dy * f + uy * 0.13 * f
                tx = sx + (dx + ux * 0.13) * f
                r = move_to_pose(pose(tx, ty, zlow), max_steps=200)
                ok = ok and near(r)
                if not near(r):
                    break
            result["pushed"] = ok
            move_to_pose(pose(sx + dx + ux * 0.13, sy + dy + uy * 0.13, zhi))
