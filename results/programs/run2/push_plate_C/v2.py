import math

base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "pushed": False}


def locate(queries):
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None


def go(x, y, z, tol, steps):
    # try once, then retry from wherever the arm stopped with looser tolerance
    r = move_to_pose(Pose((x, y, z), downward, base_frame), tolerance=tol, max_steps=steps)
    if not r.ok:
        r = move_to_pose(Pose((x, y, z), downward, base_frame), tolerance=tol * 2.0, max_steps=steps)
    return r


plate = locate(("plate", "white plate", "round plate"))
if plate is None:
    raise ValueError("plate not found")
pc = plate.geometry.pose.position
pts = plate.point_cloud.points
zmin = min(float(p[2]) for p in pts)
xs = [float(p[0]) for p in pts]
ys = [float(p[1]) for p in pts]
radius = max(max(xs) - min(xs), max(ys) - min(ys)) / 2.0
radius = min(max(radius, 0.06), 0.16)

stove = locate(("black flat stove", "stove", "cooktop"))
if stove is not None:
    sp = stove.geometry.pose.position
    tx, ty = sp[0] + 0.36, sp[1]
    result["stove_used"] = True
else:
    tx, ty = pc[0] + 0.15, pc[1] + 0.26
    result["stove_used"] = False
result["localized"] = True

dx = tx - pc[0]
dy = ty - pc[1]
dist = math.sqrt(dx * dx + dy * dy)
if dist < 0.02:
    dist = 0.02
ux, uy = dx / dist, dy / dist

push_z = zmin + 0.04
safe_z = zmin + 0.18
sx = pc[0] - ux * (radius + 0.06)
sy = pc[1] - uy * (radius + 0.06)
ex = tx - ux * (radius + 0.01)
ey = ty - uy * (radius + 0.01)

close_gripper()
r1 = go(sx, sy, safe_z, 0.03, 400)
r2 = go(sx, sy, push_z, 0.025, 300)
mx, my = (sx + ex) / 2.0, (sy + ey) / 2.0
r3 = go(mx, my, push_z, 0.03, 300)
r4 = go(ex, ey, push_z, 0.03, 300)
go(ex, ey, safe_z, 0.04, 250)

result["pushed"] = bool(r4.ok)
result["steps_ok"] = [bool(r1.ok), bool(r2.ok), bool(r3.ok), bool(r4.ok)]
result["target"] = (tx, ty)
