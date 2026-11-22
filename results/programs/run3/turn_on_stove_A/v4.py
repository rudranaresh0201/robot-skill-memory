import math

result = {"localized": False, "knob": False, "attempts": 0, "done": False}

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
    zlow = kz + 0.015
    plans = [(0.0, -1.0), (-0.02, 1.0), (0.02, -1.0), (0.0, 1.0)]
    for off, d in plans:
        result["attempts"] += 1
        x = kx + off
        y0 = ky - d * 0.10
        y1 = ky + d * 0.10
        r = move_to_pose(Pose([x, y0, zhigh], quat, frame), 0.02, 150)
        if not r.ok:
            if r.status == "terminated":
                result["done"] = True
                return
            continue
        r = move_to_pose(Pose([x, y0, zlow], quat, frame), 0.02, 120)
        if not r.ok:
            if r.status == "terminated":
                result["done"] = True
                return
            continue
        # sweep in small steps so a stall does not burn many ticks
        for i in range(1, 5):
            yy = y0 + (y1 - y0) * i / 4.0
            r = move_to_pose(Pose([x, yy, zlow], quat, frame), 0.03, 60)
            if r.status == "terminated":
                result["done"] = True
                return
            if not r.ok:
                break
        move_to_pose(Pose([x, y1, zhigh], quat, frame), 0.03, 80)

try:
    run()
except Exception as e:
    result["error"] = str(e)
