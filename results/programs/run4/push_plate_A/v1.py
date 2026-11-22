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
    c = pl.geometry.center if hasattr(pl.geometry, "center") else pl.geometry.pose.position
    px, py, pz = float(c[0]), float(c[1]), float(c[2])
    result["plate"] = [px, py, pz]
    close_gripper()
    dx, dy = 0.15, 0.26
    n = math.sqrt(dx * dx + dy * dy)
    ux, uy = dx / n, dy / n
    sx = px - ux * 0.15
    sy = py - uy * 0.15
    zlow = max(pz, 0.0) + 0.02
    zhi = 0.32
    cur = get_robot_state()
    r = move_to_pose(pose(sx, sy, zhi))
    result["above"] = near(r)
    if near(r):
        r = move_to_pose(pose(sx, sy, zlow))
        result["descend"] = near(r)
        if near(r):
            ok = True
            for f in (0.5, 1.0):
                tx = sx + dx * f
                ty = sy + dy * f
                r = move_to_pose(pose(tx, ty, zlow), max_steps=200)
                ok = ok and near(r)
                if not near(r):
                    break
            result["pushed"] = ok
            move_to_pose(pose(sx + dx, sy + dy, zhi))
