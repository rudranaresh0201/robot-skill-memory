import math

result = {"localized": False, "moved_above": False, "descended": False, "swept": False}

def near_ok(r):
    return r.ok or (r.error is not None and r.error.details.get("joint_error_rad", 9) < 0.05)

quat = [0.0, 1.0, 0.0, 0.0]

target = None
knob = localize_object("stove knob")
if knob.ok:
    kp = knob.geometry.pose.position
    kz = float(kp[2])
    result["knob_pos"] = [float(kp[0]), float(kp[1]), kz]
    if kz < 0.15:
        target = [float(kp[0]), float(kp[1]), kz]
        result["target_src"] = "knob"

if target is None:
    loc = localize_object("flat stove")
    if loc.ok:
        p = loc.geometry.pose.position
        ext = loc.geometry.extents
        result["stove_pos"] = [float(p[0]), float(p[1]), float(p[2])]
        # knob assumed near the front edge of the stove (toward the robot, smaller x)
        target = [float(p[0]) + 0.5 * float(ext[0]) - 0.03, float(p[1]), float(p[2])]
        result["target_src"] = "stove"

if target is not None:
    result["localized"] = True
    x, y, z = target
    close_gripper()
    r = move_to_pose(Pose([x, y, 0.32], quat, "robot_base"))
    result["moved_above"] = near_ok(r)
    if result["moved_above"]:
        zlow = max(z + 0.01, 0.08)
        r2 = move_to_pose(Pose([x, y - 0.04, zlow], quat, "robot_base"))
        result["descended"] = near_ok(r2)
        if result["descended"]:
            r3 = move_to_pose(Pose([x, y + 0.04, zlow], quat, "robot_base"))
            result["swept"] = near_ok(r3)
            move_to_pose(Pose([x, y + 0.04, 0.32], quat, "robot_base"))
        else:
            move_to_pose(Pose([x, y, 0.32], quat, "robot_base"))
