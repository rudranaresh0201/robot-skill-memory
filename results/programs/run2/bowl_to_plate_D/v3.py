def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]

def locate(queries, camera_name="agentview"):
    base_frame = get_robot_state().base_frame
    for q in queries:
        f = localize_object(q, camera_name=camera_name, target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None

result = {"localized": False, "grasped": False, "placed": False, "log": []}
base_frame = get_robot_state().base_frame

def mv(x, y, z, quat, tol=0.03, steps=200):
    try:
        r = move_to_pose(Pose((x, y, z), quat, base_frame), tolerance=tol, max_steps=steps)
        result["log"].append(bool(r.ok))
        return bool(r.ok)
    except Exception as e:
        result["log"].append("exc:" + str(e)[:40])
        return False

def mv_chunked(x, y, z, quat, max_chunk=0.07, tol=0.03):
    cur = get_robot_state().end_effector_poses["primary"].position
    cx, cy, cz = float(cur[0]), float(cur[1]), float(cur[2])
    d = math.sqrt((x - cx) ** 2 + (y - cy) ** 2 + (z - cz) ** 2)
    n = max(1, int(math.ceil(d / max_chunk)))
    ok = True
    for i in range(1, n + 1):
        t = i / n
        px, py, pz = cx + (x - cx) * t, cy + (y - cy) * t, cz + (z - cz) * t
        good = mv(px, py, pz, quat, tol=tol, steps=150)
        if not good:
            good = mv(px, py, pz, quat, tol=tol + 0.02, steps=250)
        ok = ok and good
    return ok

try:
    bowl = locate(("black bowl", "akita black bowl", "bowl"))
    plate = locate(("white plate", "plate", "round plate"))
    if bowl is None or plate is None:
        raise ValueError("object not found")
    result["localized"] = True

    bpts = bowl.point_cloud.points
    bcx = quantile([p[0] for p in bpts], 0.5)
    bcy = quantile([p[1] for p in bpts], 0.5)
    btop = quantile([p[2] for p in bpts], 0.95)
    rim_y = quantile([p[1] for p in bpts], 0.03)
    gx, gy = bcx, rim_y + 0.005
    ppts = plate.point_cloud.points
    pcx = quantile([p[0] for p in ppts], 0.5)
    pcy = quantile([p[1] for p in ppts], 0.5)
    psurf = quantile([p[2] for p in ppts], 0.9)

    quat = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
    open_gripper()

    gz = btop - 0.015
    mv(gx, gy, gz + 0.12, quat)
    mv(gx, gy, gz, quat, tol=0.015)
    close_gripper()

    lift_z = gz + 0.17
    ok = mv(gx, gy, lift_z, quat)
    result["grasped"] = ok

    tx = pcx + (gx - bcx)
    ty = pcy + (gy - bcy)
    moved = mv_chunked(tx, ty, lift_z, quat)
    result["transport"] = moved
    down = mv_chunked(tx, ty, psurf + 0.09, quat, max_chunk=0.05)
    result["lowered"] = down
    open_gripper()
    result["placed"] = bool(moved and down)
    mv(tx, ty, lift_z, quat)
except Exception as e:
    result["error"] = str(e)
