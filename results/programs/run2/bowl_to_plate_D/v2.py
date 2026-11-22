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
    # rim point on the low-y side (fingers straddle the wall along y)
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

    lift_z = gz + 0.15
    ok = mv(gx, gy, lift_z, quat)
    result["grasped"] = ok

    # gripper target so the bowl center ends over the plate center
    tx = pcx + (gx - bcx)
    ty = pcy + (gy - bcy)
    mv(tx, ty, lift_z, quat, steps=300)
    mv(tx, ty, psurf + 0.09, quat)
    open_gripper()
    result["placed"] = True
    mv(tx, ty, lift_z, quat)
except Exception as e:
    result["error"] = str(e)
