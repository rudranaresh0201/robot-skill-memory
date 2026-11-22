import math


def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]


base_frame = get_robot_state().base_frame
down = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "grasped": False, "placed": False}


def locate(queries):
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None


plate = locate(("white plate", "plate", "flat round plate"))
stove = locate(("flat stove", "stove", "black stove"))
if plate is None or stove is None:
    result["notes"] = "localization failed"
else:
    result["localized"] = True
    pc = plate.geometry.pose.position
    sc = stove.geometry.pose.position
    pts = plate.point_cloud.points
    top_z = quantile((p[2] for p in pts), 0.95)
    dx = sc[0] - pc[0]
    dy = sc[1] - pc[1]
    n = math.sqrt(dx * dx + dy * dy)
    if n < 1e-6:
        n = 1.0
    ux, uy = dx / n, dy / n
    push = max(0.0, n - 0.12)
    start_off = 0.13
    sx = pc[0] - ux * start_off
    sy = pc[1] - uy * start_off
    z_push = top_z - 0.01
    try:
        close_gripper()
        move_to_pose(Pose((sx, sy, z_push + 0.12), down, base_frame), tolerance=0.02, max_steps=250)
        move_to_pose(Pose((sx, sy, z_push), down, base_frame), tolerance=0.015, max_steps=250)
        steps = 3
        for i in range(1, steps + 1):
            f = push * i / steps
            move_to_pose(Pose((sx + ux * (f + 0.05), sy + uy * (f + 0.05), z_push), down, base_frame),
                         tolerance=0.02, max_steps=200)
        move_to_pose(Pose((sx + ux * (push + 0.05), sy + uy * (push + 0.05), z_push + 0.12), down, base_frame),
                     tolerance=0.02, max_steps=200)
        result["placed"] = True
    except Exception as e:
        result["notes"] = str(e)
    result["plate_center"] = tuple(pc)
    result["stove_center"] = tuple(sc)
