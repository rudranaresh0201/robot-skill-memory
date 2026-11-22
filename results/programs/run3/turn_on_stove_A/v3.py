import math

result = {"localized": False, "knob": False, "moved_above": False, "swept": False, "done": False}

def run():
    quat = [0.0, 1.0, 0.0, 0.0]
    stove = localize_object("flat stove")
    if not stove.ok:
        return
    result["localized"] = True
    sp = stove.geometry.pose.position
    frame = stove.geometry.pose.frame
    sx, sy, sz = float(sp[0]), float(sp[1]), float(sp[2])
    ex = 0.3
    try:
        ex = float(stove.geometry.extents[0])
    except Exception:
        pass
    # default guess: knob at the near edge of the stove, centered in y
    kx, ky, kz = sx - ex * 0.5, sy, sz + 0.03
    knob = localize_object("black knob")
    if knob.ok:
        kp = knob.geometry.pose.position
        dx = float(kp[0]) - sx
        dy = float(kp[1]) - sy
        if abs(dx) < 0.35 and abs(dy) < 0.35:
            kx, ky = float(kp[0]), float(kp[1])
            kz = float(kp[2])
            result["knob"] = True
    close_gripper()
    zhigh = kz + 0.20
    zlow = kz + 0.01
    r = move_to_pose(Pose([kx, ky - 0.10, zhigh], quat, frame), 0.02, 200)
    if not r.ok:
        return
    result["moved_above"] = True
    r = move_to_pose(Pose([kx, ky - 0.10, zlow], quat, frame), 0.02, 200)
    if not r.ok:
        return
    r = move_to_pose(Pose([kx, ky + 0.10, zlow], quat, frame), 0.02, 200)
    result["swept"] = bool(r.ok)
    r = move_to_pose(Pose([kx, ky + 0.10, zhigh], quat, frame), 0.02, 200)
    result["done"] = bool(r.ok)

try:
    run()
except Exception as e:
    result["error"] = str(e)
