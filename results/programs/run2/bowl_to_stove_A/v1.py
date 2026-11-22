import math

result = {"localized": False, "grasped": False, "lifted": False, "moved": False, "released": False}

def pos_of(loc):
    return list(loc.geometry.pose.position)

try:
    bowl = localize_object("black bowl")
    stove = localize_object("flat stove")
    result["localized"] = bool(bowl.ok and stove.ok)
except Exception as e:
    result["error"] = str(e)
    bowl = None
    stove = None

if result["localized"]:
    try:
        bp = pos_of(bowl)
        sp = pos_of(stove)
        st = get_robot_state()
        quat = list(st.end_effector_poses["primary"].quaternion_wxyz) if hasattr(st, "end_effector_poses") else [0, 1, 0, 0]
    except Exception:
        quat = [0, 1, 0, 0]
    frame = bowl.geometry.pose.frame
    open_gripper()
    # grasp the bowl rim: offset toward +x by bowl radius approx
    rim = [bp[0], bp[1], bp[2]]
    ext = list(bowl.geometry.extents)
    r = max(ext[0], ext[1]) / 2.0
    gx = bp[0] - r
    top = bp[2] + ext[2] / 2.0
    r1 = move_to_pose(Pose([gx, bp[1], top + 0.12], quat, frame))
    r2 = move_to_pose(Pose([gx, bp[1], top - 0.02], quat, frame))
    result["approached"] = bool(r1.ok and r2.ok)
    close_gripper()
    result["grasped"] = True
    r3 = move_to_pose(Pose([gx, bp[1], top + 0.18], quat, frame))
    result["lifted"] = bool(r3.ok)
    dx = sp[0] - bp[0]
    dy = sp[1] - bp[1]
    r4 = move_to_pose(Pose([gx + dx, bp[1] + dy, top + 0.18], quat, frame), max_steps=200)
    result["moved"] = bool(r4.ok)
    r5 = move_to_pose(Pose([gx + dx, bp[1] + dy, top + 0.07], quat, frame))
    open_gripper()
    result["released"] = True
    move_to_pose(Pose([gx + dx, bp[1] + dy, top + 0.2], quat, frame))
