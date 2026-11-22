result = {"localized": False, "grasped": False, "lifted": False, "placed": False}

def finish():
    print(result)

bowl = localize_object("black bowl")
plate = localize_object("plate")
if not (bowl.ok and plate.ok):
    finish()
else:
    result["localized"] = True
    bp = bowl.geometry.pose.position
    pp = plate.geometry.pose.position
    frame = bowl.geometry.pose.frame
    quat = [0.0, 1.0, 0.0, 0.0]
    st = get_robot_state()
    ok = True

    def goto(p, z):
        return move_to_pose(Pose([p[0], p[1], z], quat, frame), 0.02, 200)

    r = open_gripper()
    r = goto(bp, bp[2] + 0.15)
    if not r.ok:
        ok = False
    if ok:
        r = goto(bp, bp[2] + 0.03)
        ok = r.ok
    if ok:
        c = close_gripper()
        result["grasped"] = c.ok
        r = goto(bp, bp[2] + 0.2)
        ok = r.ok
        result["lifted"] = ok
    if ok:
        r = goto(pp, pp[2] + 0.2)
        ok = r.ok
    if ok:
        r = goto(pp, pp[2] + 0.1)
        ok = r.ok
    if ok:
        o = open_gripper()
        result["placed"] = o.ok
        goto(pp, pp[2] + 0.2)
    finish()
