import math

result = {"localized": False, "approached": False, "grasped": False, "lifted": False, "moved": False, "released": False}

def pos_of(loc):
    return list(loc.geometry.pose.position)

def ee_pos():
    st = get_robot_state()
    return list(st.end_effector_poses["primary"].position)

def mv(target, quat, frame, tol=0.025, seg=0.10):
    # move in short Cartesian segments to avoid long plans that time out
    fails = 0
    for i in range(14):
        try:
            cur = ee_pos()
        except Exception:
            cur = None
        if cur is None:
            nxt = target
            dist = 1.0
        else:
            d = [target[k] - cur[k] for k in range(3)]
            dist = math.sqrt(sum(x * x for x in d))
            if dist < tol:
                return True
            s = min(1.0, seg / dist)
            nxt = [cur[k] + d[k] * s for k in range(3)]
        try:
            r = move_to_pose(Pose(nxt, quat, frame), tolerance=0.015, max_steps=80)
            ok = bool(r.ok)
        except Exception as e:
            result["err_mv"] = str(e)
            ok = False
        if ok:
            fails = 0
        else:
            fails += 1
            if fails >= 2:
                break
    try:
        cur = ee_pos()
        dist = math.sqrt(sum((target[k] - cur[k]) ** 2 for k in range(3)))
        return dist < tol * 1.6
    except Exception:
        return False

bowl = None
stove = None
try:
    bowl = localize_object("black bowl")
    stove = localize_object("flat stove")
    result["localized"] = bool(bowl.ok and stove.ok)
except Exception as e:
    result["error"] = str(e)

if result["localized"]:
    quat = [0, 1, 0, 0]
    bp = pos_of(bowl)
    sp = pos_of(stove)
    frame = bowl.geometry.pose.frame
    ext = list(bowl.geometry.extents)
    r = max(ext[0], ext[1]) / 2.0
    top = bp[2] + ext[2] / 2.0
    gx = bp[0] - r + 0.015
    gy = bp[1]
    safe = top + 0.15

    try:
        open_gripper()
    except Exception:
        pass
    a1 = mv([gx, gy, safe], quat, frame)
    a2 = False
    if a1:
        a2 = mv([gx, gy, top - 0.015], quat, frame, tol=0.02)
    result["approached"] = bool(a1 and a2)
    if result["approached"]:
        try:
            close_gripper()
            result["grasped"] = True
        except Exception as e:
            result["grasp_err"] = str(e)
    if result["grasped"]:
        result["lifted"] = mv([gx, gy, safe], quat, frame)
        tx = sp[0] + (gx - bp[0])
        ty = sp[1] + (gy - bp[1])
        if result["lifted"]:
            result["moved"] = mv([tx, ty, safe], quat, frame)
        if result["moved"]:
            stove_top = sp[2] + stove.geometry.extents[2] / 2.0
            drop_z = stove_top + ext[2] + 0.03 + (top - bp[2] - ext[2] / 2.0)
            mv([tx, ty, max(drop_z, stove_top + ext[2] + 0.04)], quat, frame)
            try:
                open_gripper()
                result["released"] = True
            except Exception as e:
                result["release_err"] = str(e)
            mv([tx, ty, safe], quat, frame)
