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
        kx, ky, kz = cx - 0.12, cy - 0.01, cz
        for q in ("black stove knob", "round black knob"):
            try:
                k = localize_object(q)
                if k.ok:
                    kp = k.geometry.pose.position
                    x, y, z = float(kp[0]), float(kp[1]), float(kp[2])
                    if abs(x - cx) < 0.3 and abs(y - cy) < 0.3:
                        kx, ky, kz = x, y, z
                        result["knob_localized"] = True
                        break
            except Exception:
                pass
        close_gripper()

        def go(x, y, z):
            try:
                r = move_to_pose(Pose([x, y, z], quat, frame), tolerance=0.03, max_steps=250)
                return bool(r.ok)
            except Exception:
                return False

        ok1 = go(kx, ky - 0.09, kz + 0.15)
        result["moved"] = ok1
        if ok1:
            ok2 = go(kx, ky - 0.09, kz + 0.03)
            result["lowered"] = ok2
            if ok2:
                ok3 = go(kx, ky + 0.09, kz + 0.03)
                result["pressed"] = ok3
                go(kx, ky + 0.09, kz + 0.15)
except Exception as e:
    result["error"] = str(e)
