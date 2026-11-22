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

    def goto(x, y, z, tol=0.05, steps=200):
        return move_to_pose(Pose([x, y, z], quat, frame), tol, steps)

    # Grasp the rim on the -y side so the fingers (opening along y) straddle the wall
    gx = bp[0]
    gy = bp[1] - radius + 0.01
    ok = True
    open_gripper()
    r = goto(gx, gy, top + 0.15)
    ok = r.ok
    result["approach"] = ok
    if ok:
        r = goto(gx, gy, top + 0.03)
        ok = r.ok
    if ok:
        r = goto(gx, gy, top - 0.015, 0.06, 150)
        ok = r.ok
        result["descended"] = ok
    if ok:
        c = close_gripper()
        result["grasped"] = c.ok
        r = goto(gx, gy, top + 0.15, 0.06)
        ok = r.ok
        result["lifted"] = ok
    if ok:
        r = goto(pp[0], pp[1] - radius + 0.01, top + 0.15, 0.06)
        ok = r.ok
        result["over_plate"] = ok
    if ok:
        r = goto(pp[0], pp[1] - radius + 0.01, pp[2] + zext + 0.05, 0.06)
        ok = r.ok
    if ok:
        o = open_gripper()
        result["placed"] = o.ok
        goto(pp[0], pp[1] - radius + 0.01, top + 0.15, 0.06)
    finish()
