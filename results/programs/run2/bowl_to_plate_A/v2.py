result = {"localized": False, "grasped": False, "lifted": False, "placed": False}

def pose_at(pos, quat):
    return Pose(position=list(pos), quaternion_wxyz=list(quat), frame="robot_base")

try:
    bowl = localize_object("black bowl")
    plate = localize_object("plate")
    result["localized"] = bool(bowl.ok and plate.ok)
    if result["localized"]:
        bp = list(bowl.geometry.pose.position)
        pp = list(plate.geometry.pose.position)
        quat = [0.0, 1.0, 0.0, 0.0]
        ext_z = 0.05
        try:
            ext_z = float(bowl.geometry.extents[2])
        except Exception:
            ext_z = 0.05
        gx, gy = bp[0], bp[1]
        top = bp[2] + ext_z / 2.0
        open_gripper()
        r1 = move_to_pose(pose_at([gx, gy, top + 0.12], quat), max_steps=200)
        r2 = move_to_pose(pose_at([gx, gy, top + 0.01], quat), max_steps=200)
        close_gripper()
        result["grasped"] = bool(r2.ok)
        r3 = move_to_pose(pose_at([gx, gy, top + 0.15], quat), max_steps=200)
        result["lifted"] = bool(r3.ok)
        r4 = move_to_pose(pose_at([pp[0], pp[1], top + 0.15], quat), max_steps=250)
        r5 = move_to_pose(pose_at([pp[0], pp[1], top + 0.06], quat), max_steps=200)
        open_gripper()
        result["placed"] = bool(r4.ok and r5.ok)
        move_to_pose(pose_at([pp[0], pp[1], top + 0.15], quat), max_steps=200)
except Exception as e:
    result["error"] = str(e)
