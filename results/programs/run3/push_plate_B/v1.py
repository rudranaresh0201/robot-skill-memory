import math

def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]

def center_of(points):
    return (quantile((p[0] for p in points), 0.5),
            quantile((p[1] for p in points), 0.5),
            quantile((p[2] for p in points), 0.5))

def locate(queries):
    base = get_robot_state().base_frame
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None

result = {"localized": False, "pushed": False}
base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz

plate = locate(("white plate", "plate", "flat round plate"))
stove = locate(("black flat stove", "stove", "cooktop"))
if plate is None or stove is None:
    result["error"] = "localization failed"
else:
    result["localized"] = True
    pc = center_of(plate.point_cloud.points)
    sc = center_of(stove.point_cloud.points)
    plate_zs = [float(p[2]) for p in plate.point_cloud.points]
    push_z = min(plate_zs) + 0.025
    # target: in front of the stove (toward +x of the stove), same y
    tx, ty = sc[0] + 0.33, sc[1]
    dx, dy = tx - pc[0], ty - pc[1]
    dist = math.sqrt(dx * dx + dy * dy)
    ux, uy = dx / dist, dy / dist
    standoff = 0.15
    start = (pc[0] - ux * standoff, pc[1] - uy * standoff)
    end = (tx - ux * standoff, ty - uy * standoff)

    open_gripper()
    close_gripper()
    ok = move_to_pose(Pose((start[0], start[1], push_z + 0.15), downward, base_frame),
                      tolerance=0.02, max_steps=250).ok
    if ok:
        ok = move_to_pose(Pose((start[0], start[1], push_z), downward, base_frame),
                          tolerance=0.015, max_steps=250).ok
    result["approached"] = ok
    if ok:
        n = 5
        for i in range(1, n + 1):
            x = start[0] + (end[0] - start[0]) * i / n
            y = start[1] + (end[1] - start[1]) * i / n
            r = move_to_pose(Pose((x, y, push_z), downward, base_frame),
                             tolerance=0.02, max_steps=200)
            if not r.ok:
                result["push_fail_step"] = i
                ok = False
                break
        result["pushed"] = ok
        move_to_pose(Pose((end[0], end[1], push_z + 0.12), downward, base_frame),
                     tolerance=0.03, max_steps=150)
