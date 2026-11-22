import math

result = {"localized": False, "knob": False, "grasped": False, "rotated": 0, "done": False}

def quat_yaw(theta):
    return [0.0, math.cos(theta / 2.0), math.sin(theta / 2.0), 0.0]

def run():
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
            kx, ky, kz = float(kp[0]), float(kp[1]), float(kp[2])
            result["knob"] = True
    open_gripper()
    q0 = quat_yaw(0.0)
    r = move_to_pose(Pose([kx, ky, kz + 0.18], q0, frame), 0.05, 150)
    if not r.ok and r.status == "terminated":
        result["done"] = True
        return
    r = move_to_pose(Pose([kx, ky, kz + 0.02], q0, frame), 0.05, 100)
    if not r.ok and r.status == "terminated":
        result["done"] = True
        return
    close_gripper()
    result["grasped"] = True
    steps = 4
    for i in range(1, steps + 1):
        th = math.radians(80.0) * i / steps
        r = move_to_pose(Pose([kx, ky, kz + 0.02], quat_yaw(th), frame), 0.06, 60)
        result["rotated"] = i
        if r.status == "terminated":
            result["done"] = True
            return
    open_gripper()
    move_to_pose(Pose([kx, ky, kz + 0.18], quat_yaw(math.radians(80.0)), frame), 0.06, 80)

try:
    run()
except Exception as e:
    result["error"] = str(e)
