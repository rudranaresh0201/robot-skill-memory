import math

result = {"localized": False, "grasped": False, "moved": False, "placed": False, "log": []}


def near_ok(r):
    return r.ok or (r.error is not None and r.error.details.get("joint_error_rad", 9) < 0.05)


def mv(x, y, z, steps=250):
    p = Pose([x, y, z], [0.0, 1.0, 0.0, 0.0], "robot_base")
    r = move_to_pose(p, max_steps=steps)
    ok = near_ok(r)
    if not ok:
        result["log"].append(["fail", round(x, 3), round(y, 3), round(z, 3), str(r.status)])
    return ok


def hop(x0, y0, x1, y1, z, n=3):
    ok = True
    for i in range(1, n + 1):
        f = i / float(n)
        ok = mv(x0 + (x1 - x0) * f, y0 + (y1 - y0) * f, z)
        if not ok:
            return False
    return ok


def bowl_lifted():
    b = localize_object("black bowl")
    if not b.ok:
        return False
    z = list(b.geometry.pose.position)[2]
    result["held_z"] = z
    return z > 0.10


bowl = localize_object("black bowl")
stove = localize_object("flat stove")
if bowl.ok and stove.ok:
    bg = bowl.geometry
    sg = stove.geometry
    bc = list(bg.pose.position)
    be = list(bg.extents)
    sc = list(sg.pose.position)
    se = list(sg.extents)
    result["localized"] = True
    result["bowl"] = bc
    result["stove"] = sc
    result["bowl_ext"] = be
    full = min(be[0], be[1]) / 2.0
    top = max(bc[2] + be[2] / 2.0, 0.05)
    SAFE = 0.32
    open_gripper()
    attempts = [(0.0, 1.0, full - 0.01), (0.0, -1.0, full - 0.01),
                (0.0, 1.0, full - 0.025), (0.0, -1.0, full - 0.025)]
    ox = 0.0
    oy = 0.0
    gx = bc[0]
    gy = bc[1]
    for a in attempts:
        ox = a[0] * a[2]
        oy = a[1] * a[2]
        gx = bc[0] + ox
        gy = bc[1] + oy
        if not mv(gx, gy, SAFE):
            continue
        gz = max(top - 0.03, 0.05)
        if not mv(gx, gy, gz, steps=150):
            mv(gx, gy, SAFE)
            continue
        close_gripper()
        mv(gx, gy, SAFE)
        if bowl_lifted():
            result["grasped"] = True
            break
        open_gripper()
        mv(gx, gy, SAFE)
    if result["grasped"]:
        tx = sc[0] + ox
        ty = sc[1] + oy
        ok2 = hop(gx, gy, tx, ty, SAFE, 3)
        result["moved"] = ok2
        if ok2:
            reached = None
            for z in [0.26, 0.22, 0.19, 0.17]:
                if mv(tx, ty, z, steps=120):
                    reached = z
                else:
                    break
            result["reached_z"] = reached
            open_gripper()
            result["placed"] = reached is not None
            mv(tx, ty, SAFE, steps=150)
        else:
            open_gripper()
