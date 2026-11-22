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
    zhi = 0.32
    zlow = 0.07
    started = False
    sx = px
    sy = py
    for dx, dy in ((-0.03, -0.13), (-0.05, -0.12), (-0.03, -0.11)):
        sx = px + dx
        sy = py + dy
        r = move_to_pose(pose(sx, sy, zhi), max_steps=150)
        if not near(r):
            continue
        r = move_to_pose(pose(sx, sy, zlow), max_steps=100)
        result["descend"] = near(r)
        if near(r):
            started = True
            break
    result["started"] = started
    if started:
        ok = True
        for f in (0.25, 0.5, 0.75, 1.0):
            tx = sx + 0.04 * f
            ty = sy + 0.36 * f
            r = move_to_pose(pose(tx, ty, zlow), max_steps=100)
            if not near(r):
                ok = False
                break
        result["pushed"] = ok
        move_to_pose(pose(sx + 0.04, sy + 0.36, zhi), max_steps=150)
