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
    rad = 0.09
    try:
        ex = g.extents
        rad = max(0.06, min(0.12, float(max(ex[0], ex[1])) / 2.0))
    except Exception:
        pass
    result["rad"] = rad
    open_gripper()
    zhi = 0.32
    gx = px
    gy = py + rad - 0.015
    r = move_to_pose(pose(gx, gy, zhi), max_steps=200)
    result["above"] = near(r)
    if near(r):
        r = move_to_pose(pose(gx, gy, 0.07), max_steps=120)
        r2 = move_to_pose(pose(gx, gy, 0.03), max_steps=120)
        result["descend"] = near(r2)
        if near(r2):
            close_gripper()
            r = move_to_pose(pose(gx, gy, zhi), max_steps=150)
            result["lifted"] = near(r)
            if near(r):
                tx = gx + 0.14
                ty = gy + 0.27
                ok = True
                for f in (0.5, 1.0):
                    r = move_to_pose(pose(gx + 0.14 * f, gy + 0.27 * f, zhi), max_steps=150)
                    if not near(r):
                        ok = False
                        break
                result["carried"] = ok
                if ok:
                    r = move_to_pose(pose(tx, ty, 0.06), max_steps=150)
                    result["lowered"] = near(r)
                    open_gripper()
                    move_to_pose(pose(tx, ty, zhi), max_steps=150)
