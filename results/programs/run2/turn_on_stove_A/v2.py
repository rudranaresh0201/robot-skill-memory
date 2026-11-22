import math

result = {"localized": False, "knob_localized": False, "moved": False, "pressed": False}

try:
    loc = localize_object("flat stove")
    result["localized"] = bool(loc.ok)
    if loc.ok:
        p = loc.geometry.pose.position
        cx, cy, cz = float(p[0]), float(p[1]), float(p[2])
        frame = get_robot_state().base_frame
        quat = [0.0, 1.0, 0.0, 0.0]
        # default knob guess: on the robot-facing side of the stove
        kx, ky, kz = cx - 0.12, cy - 0.01, cz
        for q in ("black stove knob", "round black knob"):
            try:
                k = localize_object(q)
                if k.ok:
                    kp = k.geometry.pose.position
                    x, y, z = float(kp[0]), float(kp[1]), float(kp[2])
                    if abs(x - cx) < 0.25 and abs(y - cy) < 0.25:
                        kx, ky, kz = x, y, z
                        result["knob_localized"] = True
                        break
            except Exception:
                pass
        close_gripper()

        def go(x, y, z):
            return move_to_pose(Pose([x, y, z], quat, frame), tolerance=0.02, max_steps=200)

        r = go(kx, ky - 0.08, kz + 0.2)
        result["moved"] = bool(r.ok)
        for h in (0.04, 0.02):
            go(kx, ky - 0.08, kz + h)
            go(kx, ky + 0.08, kz + h)
            go(kx, ky + 0.08, kz + 0.15)
            go(kx, ky - 0.08, kz + 0.15)
            result["pressed"] = True
except Exception as e:
    result["error"] = str(e)
