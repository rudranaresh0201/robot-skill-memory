result = {"localized": False, "grasped": False, "lifted": False, "moved": False, "placed": False}

def run():
    bowl = localize_object("black bowl")
    if not bowl.ok:
        return
    # The scene swaps names: the wooden cabinet may be matched by "stove".
    # Pick the low, flat candidate.
    stove = None
    best_h = 99.0
    for qry in ["flat stove burner", "black burner plate", "flat stove", "stove"]:
        s = localize_object(qry)
        if s.ok:
            h = list(s.geometry.extents)[2]
            if h < best_h:
                best_h = h
                stove = s
            if h < 0.1:
                break
    if stove is None:
        return
    result["localized"] = True
    result["stove_h"] = best_h

    bp = list(bowl.geometry.pose.position)
    sp = list(stove.geometry.pose.position)
    frame = bowl.geometry.pose.frame
    ext = list(bowl.geometry.extents)
    sext = list(stove.geometry.extents)
    rad = min(ext[0], ext[1]) / 2.0
    top = bp[2] + ext[2] / 2.0
    q = [0.0, 0.70710678, 0.70710678, 0.0]
    tol = 0.05

    gx = bp[0] - rad + 0.01
    gy = bp[1]
    gz = top - 0.02

    r = open_gripper()
    if not r.ok:
        return
    r = move_to_pose(Pose([gx, gy, top + 0.12], q, frame), tolerance=tol, max_steps=250)
    if not r.ok:
        result["fail"] = "approach"
        return
    r = move_to_pose(Pose([gx, gy, gz], q, frame), tolerance=tol, max_steps=200)
    if not r.ok:
        result["fail"] = "descend"
        return
    r = close_gripper()
    if not r.ok:
        return
    result["grasped"] = True

    lift_z = top + 0.12
    r = move_to_pose(Pose([gx, gy, lift_z], q, frame), tolerance=tol, max_steps=200)
    if not r.ok:
        result["fail"] = "lift"
        return
    result["lifted"] = True

    tx = sp[0] - rad + 0.01
    ty = sp[1]
    n = 6
    for i in range(1, n + 1):
        f = i / float(n)
        wx = gx + (tx - gx) * f
        wy = gy + (ty - gy) * f
        r = move_to_pose(Pose([wx, wy, lift_z], q, frame), tolerance=0.1, max_steps=250)
        if not r.ok:
            result["fail"] = "move%d" % i
            # retry once with a slightly higher target
            r = move_to_pose(Pose([wx, wy, lift_z + 0.05], q, frame), tolerance=0.12, max_steps=250)
            if not r.ok:
                return
    result["moved"] = True

    stove_top = sp[2] + sext[2] / 2.0
    offset = ext[2] - 0.02
    rel_z = max(stove_top + offset + 0.04, 0.08)
    r = move_to_pose(Pose([tx, ty, rel_z], q, frame), tolerance=0.08, max_steps=200)
    if not r.ok:
        result["fail"] = "lower"
    r = open_gripper()
    result["placed"] = r.ok
    move_to_pose(Pose([tx, ty, lift_z + 0.08], q, frame), tolerance=0.1, max_steps=200)

try:
    run()
except Exception as e:
    result["error"] = str(e)
