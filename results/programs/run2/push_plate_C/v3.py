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
    pose = Pose((x, y, z), downward, base_frame)
    last = None
    for t in (tol, tol * 2.0, tol * 3.0):
        last = move_to_pose(pose, tolerance=t, max_steps=steps)
        if last.ok:
            return last
    return last


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
px, py = float(pc[0]), float(pc[1])

# target: just in front of stove (toward +x of robot base), same y as stove
stove = locate(("black flat stove", "stove", "cooktop"))
if stove is not None:
    sp = stove.geometry.pose.position
    tx, ty = float(sp[0]) + 0.33, float(sp[1])
    result["stove_used"] = True
else:
    tx, ty = px + 0.12, py + 0.25
    result["stove_used"] = False
result["localized"] = True

dx = tx - px
dy = ty - py
dist = math.sqrt(dx * dx + dy * dy)
if dist < 0.02:
    dist = 0.02
ux, uy = dx / dist, dy / dist

push_z = zmin + 0.03
safe_z = zmin + 0.20
sx = px - ux * (radius + 0.05)
sy = py - uy * (radius + 0.05)
ex = tx - ux * (radius + 0.01)
ey = ty - uy * (radius + 0.01)

close_gripper()
r0 = go(sx, sy, safe_z + 0.1, 0.04, 500)
r1 = go(sx, sy, safe_z, 0.04, 400)
r2 = go(sx, sy, push_z, 0.03, 400)
mx, my = (sx + ex) / 2.0, (sy + ey) / 2.0
r3 = go(mx, my, push_z, 0.03, 400)
r4 = go(ex, ey, push_z, 0.03, 400)
go(ex, ey, safe_z, 0.05, 300)

result["pushed"] = bool(r4.ok)
result["steps_ok"] = [bool(r0.ok), bool(r1.ok), bool(r2.ok), bool(r3.ok), bool(r4.ok)]
result["target"] = (tx, ty)
