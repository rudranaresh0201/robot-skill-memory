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
    push_z = min(plate_zs) + 0.035
    hover_z = push_z + 0.12
    # stove front region lies ~0.15 m further in +x than the plate start, at stove y
    tx = pc[0] + 0.17
    ty = sc[1]
    result["plate_center"] = list(pc)
    result["target"] = [tx, ty]

    def go(x, y, z, tol=0.02, steps=200):
        return move_to_pose(Pose((x, y, z), downward, base_frame),
                            tolerance=tol, max_steps=steps)

    close_gripper()

    dx = tx - pc[0]
    dy = ty - pc[1]
    dist = math.sqrt(dx * dx + dy * dy)
    ux, uy = dx / dist, dy / dist
    standoff = 0.12
    sx, sy = pc[0] - ux * standoff, pc[1] - uy * standoff
    ex, ey = tx - ux * standoff, ty - uy * standoff

    ok = False
    r = go(sx, sy, hover_z, 0.03, 250)
    result["hover"] = r.ok
    if r.ok:
        r = go(sx, sy, push_z, 0.02, 200)
        result["descend"] = r.ok
        if r.ok:
            ok = True
            n = 5
            for i in range(1, n + 1):
                x = sx + (ex - sx) * i / n
                y = sy + (ey - sy) * i / n
                r = go(x, y, push_z, 0.025, 150)
                if not r.ok:
                    ok = False
                    result["failed_step"] = i
                    break
            go(ex, ey, hover_z, 0.04, 150)
    result["pushed"] = ok
