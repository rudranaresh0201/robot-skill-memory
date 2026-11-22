import math

result = {"localized": False, "approached": False, "grasped": False, "lifted": False, "moved": False, "released": False}

def pos_of(loc):
    return list(loc.geometry.pose.position)

def mv(p, quat, frame, tol=0.02, steps=250):
    ok = False
    for i in range(3):
        try:
            r = move_to_pose(Pose(p, quat, frame), tolerance=tol, max_steps=steps)
            ok = bool(r.ok)
        except Exception as e:
            result["err_mv"] = str(e)
            ok = False
        if ok:
            break
    return ok

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
    try:
        st = get_robot_state()
        quat = list(st.end_effector_poses["primary"].quaternion_wxyz)
    except Exception:
        quat = [0, 1, 0, 0]
    bp = pos_of(bowl)
    sp = pos_of(stove)
    frame = bowl.geometry.pose.frame
    ext = list(bowl.geometry.extents)
    r = max(ext[0], ext[1]) / 2.0
    top = bp[2] + ext[2] / 2.0
    gx = bp[0] - r + 0.015
    gy = bp[1]
    safe = top + 0.20

    try:
        open_gripper()
    except Exception:
        pass
    a1 = mv([gx, gy, safe], quat, frame)
    a2 = mv([gx, gy, top - 0.015], quat, frame, tol=0.015)
    result["approached"] = bool(a1 and a2)
    try:
        close_gripper()
        result["grasped"] = True
    except Exception as e:
        result["grasp_err"] = str(e)
    l = mv([gx, gy, safe], quat, frame)
    result["lifted"] = l
    tx = sp[0] + (gx - bp[0])
    ty = sp[1] + (gy - bp[1])
    mid_x = (gx + tx) / 2.0
    mid_y = (gy + ty) / 2.0
    m1 = mv([mid_x, mid_y, safe], quat, frame)
    m2 = mv([tx, ty, safe], quat, frame)
    result["moved"] = bool(m1 and m2)
    stove_top = sp[2] + stove.geometry.extents[2] / 2.0
    drop_z = stove_top + ext[2] + 0.03 + (top - bp[2] - ext[2] / 2.0)
    mv([tx, ty, max(drop_z, stove_top + ext[2] + 0.04)], quat, frame)
    try:
        open_gripper()
        result["released"] = True
    except Exception as e:
        result["release_err"] = str(e)
    mv([tx, ty, safe], quat, frame)
