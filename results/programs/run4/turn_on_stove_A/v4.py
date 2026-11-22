import math

result = {"localized": False, "sweeps": []}

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
        target = [float(p[0]) + 0.5 * float(ext[0]) - 0.03, float(p[1]), float(p[2])]
        result["target_src"] = "stove"

if target is not None:
    result["localized"] = True
    x, y, z = target
    close_gripper()
    plans = [
        (0.025, 0.04, 1),
        (-0.025, 0.04, -1),
        (0.025, 0.03, -1),
        (-0.025, 0.03, 1),
        (0.0, 0.035, 1),
        (0.0, 0.035, -1),
    ]
    for dx, zl, d in plans:
        info = {"dx": dx, "z": zl, "dir": d}
        ys = y - 0.07 * d
        ye = y + 0.07 * d
        r = move_to_pose(Pose([x + dx, ys, 0.30], quat, "robot_base"))
        if not near_ok(r):
            info["fail"] = "above"
            result["sweeps"].append(info)
            continue
        r2 = move_to_pose(Pose([x + dx, ys, zl], quat, "robot_base"))
        info["down"] = near_ok(r2)
        if info["down"]:
            r3 = move_to_pose(Pose([x + dx, ye, zl], quat, "robot_base"))
            info["swept"] = near_ok(r3)
            move_to_pose(Pose([x + dx, ye, 0.30], quat, "robot_base"))
        else:
            move_to_pose(Pose([x + dx, ys, 0.30], quat, "robot_base"))
        result["sweeps"].append(info)
