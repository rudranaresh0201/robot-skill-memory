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
    # (x_start, y_start, x_end, y_end, z_level)
    plans = []
    for zl in [0.05, 0.035]:
        for dx in [0.03, -0.03]:
            plans.append((x + dx, y - 0.07, x + dx, y + 0.07, zl))
            plans.append((x + dx, y + 0.07, x + dx, y - 0.07, zl))
        for dy in [0.03, -0.03]:
            plans.append((x - 0.07, y + dy, x + 0.07, y + dy, zl))
            plans.append((x + 0.07, y + dy, x - 0.07, y + dy, zl))
    fails = 0
    for (xs, ys, xe, ye, zl) in plans:
        if fails >= 3:
            break
        info = {"s": [round(xs, 3), round(ys, 3)], "z": zl}
        r = move_to_pose(Pose([xs, ys, 0.30], quat, "robot_base"), max_steps=200)
        if not near_ok(r):
            info["fail"] = "above"
            fails += 1
            result["sweeps"].append(info)
            continue
        r2 = move_to_pose(Pose([xs, ys, zl], quat, "robot_base"), max_steps=150)
        info["down"] = near_ok(r2)
        if info["down"]:
            r3 = move_to_pose(Pose([xe, ye, zl], quat, "robot_base"), max_steps=150)
            info["swept"] = near_ok(r3)
            if not info["swept"]:
                fails += 1
            move_to_pose(Pose([xe, ye, 0.30], quat, "robot_base"), max_steps=150)
        else:
            fails += 1
            move_to_pose(Pose([xs, ys, 0.30], quat, "robot_base"), max_steps=150)
        result["sweeps"].append(info)
