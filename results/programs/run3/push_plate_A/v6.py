import math

result = {"localized": False, "pushed": False}

plate = localize_object("plate")
stove = localize_object("flat stove")
if not (plate.ok and stove.ok):
    result["error"] = "localization failed"
else:
    result["localized"] = True
    pp = list(plate.geometry.pose.position)
    sp = list(stove.geometry.pose.position)
    tx, ty = sp[0] + 0.12, sp[1]
    dx, dy = tx - pp[0], ty - pp[1]
    n = math.sqrt(dx * dx + dy * dy)
    result["plate_pos"] = pp
    result["target"] = [tx, ty]
    if n < 1e-3:
        result["pushed"] = True
    else:
        dx, dy = dx / n, dy / n
        q = [0.0, 1.0, 0.0, 0.0]

        def go(x, y, z, tol=0.05, ms=200):
            return move_to_pose(Pose([x, y, z], q, "robot_base"),
                                tolerance=tol, max_steps=ms)

        close_gripper()
        zlow = None
        sx = sy = off = None
        tries = []
        for o in (0.11, 0.08, 0.14, 0.06):
            cx, cy = pp[0] - dx * o, pp[1] - dy * o
            r = go(cx, cy, pp[2] + 0.25, 0.06, 250)
            if not r.ok:
                tries.append([o, "above_fail"])
                continue
            for dz in (0.06, 0.08, 0.11):
                r = go(cx, cy, pp[2] + dz, 0.06, 150)
                tries.append([o, dz, r.ok])
                if r.ok:
                    zlow = pp[2] + dz
                    sx, sy, off = cx, cy, o
                    break
            if zlow is not None:
                break
            go(cx, cy, pp[2] + 0.25, 0.08, 150)
        result["down_try"] = tries
        result["above"] = zlow is not None
        if zlow is not None:
            dist = n + off - 0.1
            if dist < 0.05:
                dist = 0.05
            steps = 5
            good = True
            for i in range(1, steps + 1):
                f = i / steps
                r = go(sx + dx * dist * f, sy + dy * dist * f, zlow, 0.06, 200)
                if not r.ok:
                    good = False
                    result["push_fail_step"] = i
                    break
            result["pushed"] = good
            up = go(sx + dx * dist, sy + dy * dist, pp[2] + 0.2, 0.06, 200)
            result["retreated"] = up.ok
