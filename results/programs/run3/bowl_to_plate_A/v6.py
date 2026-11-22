result = {"localized": False, "grasped": False, "lifted": False, "placed": False}

def finish():
    print(result)

bowl = localize_object("bowl")
plate = localize_object("plate")
if not (bowl.ok and plate.ok):
    finish()
else:
    result["localized"] = True
    bp = list(bowl.geometry.pose.position)
    pp = list(plate.geometry.pose.position)
    frame = bowl.geometry.pose.frame
    quat = [0.0, 1.0, 0.0, 0.0]
    radius = 0.055
    zext = 0.05
    try:
        ext = list(bowl.geometry.extents)
        radius = max(0.04, min(0.09, ext[0] / 2.0))
        zext = max(0.03, min(0.1, ext[2]))
    except Exception:
        pass
    result["radius"] = radius
    top = bp[2] + zext / 2.0

    def goto(x, y, z, tol=0.05, steps=300):
        return move_to_pose(Pose([x, y, z], quat, frame), tol, steps)

    gx = bp[0]
    gy = bp[1] - radius + 0.01
    open_gripper()
    r = goto(gx, gy, top + 0.15)
    ok = r.ok
    result["approach"] = ok
    if ok:
        r = goto(gx, gy, top + 0.03)
        ok = r.ok
    if ok:
        r = goto(gx, gy, top - 0.015, 0.06, 200)
        ok = r.ok
        result["descended"] = ok
    if ok:
        c = close_gripper()
        result["grasped"] = c.ok
        hz = top + 0.18
        r = goto(gx, gy, hz, 0.06)
        result["lifted"] = r.ok
        tx = pp[0]
        ty = pp[1] - radius + 0.01
        n = 5
        for i in range(1, n + 1):
            f = i / float(n)
            r = goto(gx + (tx - gx) * f, gy + (ty - gy) * f, hz, 0.1, 300)
            result["hop%d" % i] = r.ok
        r = goto(tx, ty, pp[2] + zext + 0.08, 0.1, 300)
        result["lowered"] = r.ok
        o = open_gripper()
        result["placed"] = o.ok
        goto(tx, ty, hz, 0.1, 300)
    finish()
