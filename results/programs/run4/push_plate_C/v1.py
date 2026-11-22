import math

base_frame = get_robot_state().base_frame
down = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "pushed": False}


def near(r):
    return r.ok or (r.error is not None and r.error.details.get("joint_error_rad", 9) < 0.05)


def locate(queries):
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None


plate = locate(("white plate", "plate", "round plate"))
stove = locate(("flat stove", "black stove", "stove"))
if plate is None or stove is None:
    result["error"] = "localization failed"
else:
    pc = plate.geometry.pose.position
    sc = stove.geometry.pose.position
    zs = sorted(float(p[2]) for p in plate.point_cloud.points)
    plate_z = zs[len(zs) // 2]
    ext = plate.geometry.extents
    radius = max(0.08, min(0.14, 0.5 * max(float(ext[0]), float(ext[1]))))
    # stove front = table +x (robot forward) of stove, about 0.36 m
    tx, ty = sc[0] + 0.36, sc[1] + 0.0
    dx, dy = tx - pc[0], ty - pc[1]
    dist = math.sqrt(dx * dx + dy * dy)
    result["localized"] = True
    result["plate"] = [float(v) for v in pc]
    result["stove"] = [float(v) for v in sc]
    result["target"] = [tx, ty]
    if dist < 0.02:
        result["pushed"] = True
    else:
        ux, uy = dx / dist, dy / dist
        off = radius + 0.04
        sx, sy = pc[0] - ux * off, pc[1] - uy * off
        push_z = plate_z + 0.0
        if push_z < 0.0:
            push_z = plate_z
        hi = 0.30
        close_gripper()
        ok = near(move_to_pose(Pose((sx, sy, hi), down, base_frame), tolerance=0.02, max_steps=250))
        ok = ok and near(move_to_pose(Pose((sx, sy, push_z + 0.01), down, base_frame), tolerance=0.02, max_steps=250))
        result["at_start"] = ok
        if ok:
            steps = 3
            good = True
            for i in range(1, steps + 1):
                f = dist * i / steps
                r = move_to_pose(Pose((sx + ux * f, sy + uy * f, push_z + 0.01), down, base_frame),
                                 tolerance=0.02, max_steps=200)
                if not near(r):
                    good = False
                    break
            result["pushed"] = good
            move_to_pose(Pose((sx + ux * dist, sy + uy * dist, hi), down, base_frame),
                         tolerance=0.03, max_steps=250)
