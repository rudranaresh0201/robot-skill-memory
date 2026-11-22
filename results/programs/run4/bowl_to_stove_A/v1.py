import math

result = {"localized": False, "grasped": False, "moved": False, "placed": False}


def near_ok(r):
    return r.ok or (r.error is not None and r.error.details.get("joint_error_rad", 9) < 0.05)


def mv(x, y, z):
    p = Pose([x, y, z], [0.0, 1.0, 0.0, 0.0], "robot_base")
    r = move_to_pose(p)
    return near_ok(r)


def center_of(loc):
    g = loc.geometry
    return list(g.pose.position), list(g.extents)


bowl = localize_object("black bowl")
stove = localize_object("flat stove")
if bowl.ok and stove.ok:
    bc, be = center_of(bowl)
    sc, se = center_of(stove)
    result["localized"] = True
    result["bowl"] = bc
    result["stove"] = sc
    rim_off = max(0.0, be[0] / 2.0 - 0.01)
    top = bc[2] + be[2] / 2.0
    gz = top - 0.02
    gx = bc[0] + rim_off
    gy = bc[1]
    SAFE = 0.32
    open_gripper()
    ok = mv(gx, gy, SAFE)
    ok = ok and mv(gx, gy, gz)
    if ok:
        close_gripper()
        result["grasped"] = True
        ok2 = mv(gx, gy, SAFE)
        tx = sc[0] + rim_off
        ty = sc[1]
        ok2 = ok2 and mv(tx, ty, SAFE)
        result["moved"] = ok2
        if ok2:
            dz = sc[2] + se[2] / 2.0 + be[2] + 0.03
            dz = max(dz, 0.12)
            d = mv(tx, ty, dz)
            open_gripper()
            result["placed"] = d
            mv(tx, ty, SAFE)
    else:
        mv(gx, gy, SAFE)
