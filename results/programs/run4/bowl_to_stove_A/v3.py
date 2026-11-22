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
    rim_off = max(0.0, min(be[0], be[1]) / 2.0 - 0.01)
    top = bc[2] + be[2] / 2.0
    SAFE = 0.32
    open_gripper()
    # the bottle sits on the +x side of the bowl, so grasp on the -x side first
    sides = [(-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)]
    grasp_ok = False
    gx = bc[0]
    gy = bc[1]
    ox = 0.0
    oy = 0.0
    for s in sides:
        ox = s[0] * rim_off
        oy = s[1] * rim_off
        gx = bc[0] + ox
        gy = bc[1] + oy
        if not mv(gx, gy, SAFE):
            continue
        for gz in [max(top - 0.025, 0.05), max(top - 0.015, 0.06), top + 0.01]:
            if mv(gx, gy, gz):
                grasp_ok = True
                break
            mv(gx, gy, SAFE)
        if grasp_ok:
            break
    if grasp_ok:
        close_gripper()
        result["grasped"] = True
        ok2 = mv(gx, gy, SAFE)
        tx = sc[0] + ox
        ty = sc[1] + oy
        ok2 = ok2 and mv(tx, ty, SAFE)
        result["moved"] = ok2
        if ok2:
            stove_top = sc[2] + se[2] / 2.0
            dz = max(stove_top + be[2] + 0.04, 0.12)
            d = mv(tx, ty, dz)
            open_gripper()
            result["placed"] = d
            mv(tx, ty, SAFE)
    else:
        mv(gx, gy, SAFE)
