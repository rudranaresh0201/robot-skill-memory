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
    # stove front is roughly +x of the stove in the robot base frame
    tx, ty = sp[0] + 0.36, sp[1]
    dx, dy = tx - pp[0], ty - pp[1]
    n = math.sqrt(dx * dx + dy * dy)
    if n < 1e-3:
        result["pushed"] = True
    else:
        dx, dy = dx / n, dy / n
        q = [0.0, 1.0, 0.0, 0.0]
        sx, sy = pp[0] - dx * 0.14, pp[1] - dy * 0.14
        zlow = pp[2] + 0.04
        zhigh = pp[2] + 0.2

        def go(x, y, z):
            return move_to_pose(Pose([x, y, z], q, "robot_base"), tolerance=0.02, max_steps=200)

        ok = close_gripper().ok
        r = go(sx, sy, zhigh)
        result["above"] = r.ok
        if r.ok:
            r = go(sx, sy, zlow)
            result["down"] = r.ok
            if r.ok:
                dist = n
                steps = 4
                good = True
                for i in range(1, steps + 1):
                    f = i / steps
                    r = go(sx + dx * (dist) * f, sy + dy * (dist) * f, zlow)
                    if not r.ok:
                        good = False
                        result["push_fail_step"] = i
                        break
                result["pushed"] = good
        go_up = go(sx + dx * n, sy + dy * n, zhigh)
        result["retreated"] = go_up.ok
