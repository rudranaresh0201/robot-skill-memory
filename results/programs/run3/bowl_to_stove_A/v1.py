result = {"localized": False, "grasped": False, "lifted": False, "moved": False, "placed": False}

def run():
    bowl = localize_object("black bowl")
    if not bowl.ok:
        return
    stove = localize_object("flat stove")
    if not stove.ok:
        return
    result["localized"] = True

    bp = list(bowl.geometry.pose.position)
    sp = list(stove.geometry.pose.position)
    frame = bowl.geometry.pose.frame
    q = [0.0, 1.0, 0.0, 0.0]

    r = open_gripper()
    if not r.ok:
        return

    # Approach above the bowl rim (grasp the rim edge via center top)
    r = move_to_pose(Pose([bp[0], bp[1], bp[2] + 0.15], q, frame), max_steps=200)
    if not r.ok:
        return
    r = move_to_pose(Pose([bp[0], bp[1], bp[2] + 0.04], q, frame), max_steps=200)
    if not r.ok:
        return
    r = close_gripper()
    if not r.ok:
        return
    result["grasped"] = True

    r = move_to_pose(Pose([bp[0], bp[1], bp[2] + 0.18], q, frame), max_steps=200)
    if not r.ok:
        return
    result["lifted"] = True

    r = move_to_pose(Pose([sp[0], sp[1], bp[2] + 0.18], q, frame), max_steps=250)
    if not r.ok:
        return
    result["moved"] = True

    r = move_to_pose(Pose([sp[0], sp[1], sp[2] + 0.12], q, frame), max_steps=200)
    if not r.ok:
        return
    r = open_gripper()
    result["placed"] = r.ok
    move_to_pose(Pose([sp[0], sp[1], sp[2] + 0.25], q, frame), max_steps=200)

try:
    run()
except Exception as e:
    result["error"] = str(e)
