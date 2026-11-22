result = {"localized": False, "grasped": False, "lifted": False, "moved": False, "placed": False}

def run():
    bowl = localize_object("black bowl")
    if not bowl.ok:
        return
    stove = localize_object("flat stove")
    if not stove.ok:
        return
    result["localized"] = True

    bp = list(bowl.geometry.pose.position)
    sp = list(stove.geometry.pose.position)
    frame = bowl.geometry.pose.frame
    ext = list(bowl.geometry.extents)
    rad = min(ext[0], ext[1]) / 2.0
    top = bp[2] + ext[2] / 2.0
    # gripper down, fingers separated along x so they straddle the rim
    q = [0.0, 0.70710678, 0.70710678, 0.0]
    tol = 0.05

    gx = bp[0] - rad + 0.01
    gy = bp[1]
    gz = top - 0.02
    result["grasp_z"] = gz

    r = open_gripper()
    if not r.ok:
        return
    r = move_to_pose(Pose([gx, gy, top + 0.12], q, frame), tolerance=tol, max_steps=250)
    if not r.ok:
        result["fail"] = "approach"
        return
    r = move_to_pose(Pose([gx, gy, gz], q, frame), tolerance=tol, max_steps=200)
    if not r.ok:
        result["fail"] = "descend"
        return
    r = close_gripper()
    if not r.ok:
        return
    result["grasped"] = True

    r = move_to_pose(Pose([gx, gy, top + 0.15], q, frame), tolerance=tol, max_steps=200)
    if not r.ok:
        result["fail"] = "lift"
        return
    result["lifted"] = True

    tx = sp[0] - rad + 0.01
    ty = sp[1]
    r = move_to_pose(Pose([tx, ty, top + 0.15], q, frame), tolerance=tol, max_steps=300)
    if not r.ok:
        result["fail"] = "move"
        return
    result["moved"] = True

    r = move_to_pose(Pose([tx, ty, sp[2] + ext[2] + 0.08], q, frame), tolerance=tol, max_steps=200)
    if not r.ok:
        result["fail"] = "lower"
        return
    r = open_gripper()
    result["placed"] = r.ok
    move_to_pose(Pose([tx, ty, sp[2] + ext[2] + 0.25], q, frame), tolerance=tol, max_steps=200)

try:
    run()
except Exception as e:
    result["error"] = str(e)
