import math

result = {"localized": False, "moved_above": False, "pressed": False, "done": False}

def run():
    loc = localize_object("flat stove")
    if not loc.ok:
        return
    result["localized"] = True
    c = loc.geometry.pose.position
    x, y, z = float(c[0]), float(c[1]), float(c[2])
    frame = loc.geometry.pose.frame
    ex = 0.2
    try:
        ex = float(loc.geometry.extents[0])
    except Exception:
        pass
    quat = [0.0, 1.0, 0.0, 0.0]
    close_gripper()
    # knob assumed on the front edge (toward the robot) of the stove
    kx = x - ex * 0.5
    r = move_to_pose(Pose([kx, y - 0.10, z + 0.20], quat, frame), 0.02, 200)
    if not r.ok:
        return
    result["moved_above"] = True
    r = move_to_pose(Pose([kx, y - 0.10, z + 0.05], quat, frame), 0.02, 200)
    if not r.ok:
        return
    # sweep sideways to rotate the knob
    r = move_to_pose(Pose([kx, y + 0.10, z + 0.05], quat, frame), 0.02, 200)
    result["pressed"] = bool(r.ok)
    r = move_to_pose(Pose([kx, y + 0.10, z + 0.20], quat, frame), 0.02, 200)
    result["done"] = bool(r.ok)

try:
    run()
except Exception as e:
    result["error"] = str(e)
