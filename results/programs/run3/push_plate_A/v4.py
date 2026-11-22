import math

result = {"localized": False, "pushed": False}

def fail(msg):
    result["error"] = msg
    print(msg)

plate = localize_object("plate")
stove = localize_object("flat stove")
if not (plate.ok and stove.ok):
    fail("localization failed")
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
        off = 0.13
        sx, sy = pp[0] - dx * off, pp[1] - dy * off
        zhigh = pp[2] + 0.2

        def go(x, y, z):
            # Looser tolerance and larger step budget: previous runs stalled
            # at ~0.05-0.08 rad error just short of the 0.03 tolerance.
            return move_to_pose(Pose([x, y, z], q, "robot_base"),
                                tolerance=0.09, max_steps=300)

        close_gripper()
        r = go(sx, sy, zhigh)
        result["above"] = r.ok
        if r.ok:
            r = None
            zlow = None
            for dz in (0.08, 0.06, 0.1):
                zl = pp[2] + dz
                r = go(sx, sy, zl)
                result["down"] = r.ok
                if r.ok:
                    zlow = zl
                    result["zlow"] = zl
                    break
            if r is not None and r.ok:
                dist = n + off - 0.1
                if dist < 0.05:
                    dist = 0.05
                steps = 5
                good = True
                for i in range(1, steps + 1):
                    f = i / steps
                    r = go(sx + dx * dist * f, sy + dy * dist * f, zlow)
                    if not r.ok:
                        good = False
                        result["push_fail_step"] = i
                        break
                result["pushed"] = good
                ex = sx + dx * dist
                ey = sy + dy * dist
                up = go(ex, ey, zhigh)
                result["retreated"] = up.ok
