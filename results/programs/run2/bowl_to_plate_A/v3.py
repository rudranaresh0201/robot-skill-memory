result = {"localized": False, "grasped": False, "lifted": False, "placed": False}

def pose_at(pos, quat):
    return Pose(position=list(pos), quaternion_wxyz=list(quat), frame="robot_base")

def pick_bowl():
    best = None
    for q in ["akita black bowl", "bowl", "black bowl"]:
        try:
            r = localize_object(q)
        except Exception:
            continue
        if not r.ok:
            continue
        e = [float(v) for v in r.geometry.extents]
        # bowl: roughly 0.1-0.2 m wide, under ~0.1 m tall
        if 0.08 < max(e[0], e[1]) < 0.22 and e[2] < 0.1:
            return r
        if best is None:
            best = r
    return best

try:
    bowl = pick_bowl()
    plate = localize_object("plate")
    result["localized"] = bool(bowl is not None and bowl.ok and plate.ok)
    if result["localized"]:
        bp = list(bowl.geometry.pose.position)
        pp = list(plate.geometry.pose.position)
        ext = [float(v) for v in bowl.geometry.extents]
        r = min(ext[0], ext[1]) / 2.0
        r = max(0.04, min(r, 0.1))
        quat = [0.0, 1.0, 0.0, 0.0]
        top = bp[2] + ext[2] / 2.0
        gx = bp[0]
        gy = bp[1] + r - 0.012
        open_gripper()
        move_to_pose(pose_at([gx, gy, top + 0.12], quat), max_steps=200)
        r2 = move_to_pose(pose_at([gx, gy, top - 0.02], quat), max_steps=200)
        close_gripper()
        result["grasped"] = bool(r2.ok)
        r3 = move_to_pose(pose_at([gx, gy, top + 0.18], quat), max_steps=200)
        result["lifted"] = bool(r3.ok)
        tx = pp[0]
        ty = pp[1] + r - 0.012
        r4 = move_to_pose(pose_at([tx, ty, top + 0.18], quat), max_steps=250)
        r5 = move_to_pose(pose_at([tx, ty, top + 0.07], quat), max_steps=200)
        open_gripper()
        result["placed"] = bool(r4.ok and r5.ok)
        move_to_pose(pose_at([tx, ty, top + 0.2], quat), max_steps=200)
except Exception as e:
    result["error"] = str(e)
