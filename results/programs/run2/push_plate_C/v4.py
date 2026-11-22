import math

base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "pushed": False, "rim_grasp": False}


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

stove = locate(("black flat stove", "stove", "cooktop"))
if stove is not None:
    sp = stove.geometry.pose.position
    tx, ty = float(sp[0]) + 0.30, float(sp[1])
    result["stove_used"] = True
else:
    tx, ty = px + 0.12, py + 0.25
    result["stove_used"] = False
result["localized"] = True
result["target"] = (tx, ty)
result["plate"] = (px, py, zmin, radius)

# ---- Strategy A: grasp front rim (far from base), carry to target ----
open_gripper()
gx = max(xs) - 0.015
gy = py
grasp_z = zmin + 0.012
hover = zmin + 0.18
a0 = go(gx, gy, hover, 0.03, 400)
a1 = go(gx, gy, grasp_z + 0.02, 0.02, 400)
ok_down = bool(a1.ok)
result["rim_steps"] = [bool(a0.ok), ok_down]
if ok_down:
    close_gripper()
    b0 = go(gx, gy, hover, 0.03, 400)
    # place so plate center lands at target: shift by rim offset
    ox = tx + (gx - px)
    oy = ty + (gy - py)
    b1 = go(ox, oy, hover, 0.04, 500)
    b2 = go(ox, oy, zmin + 0.05, 0.03, 400)
    open_gripper()
    go(ox, oy, hover, 0.05, 300)
    result["rim_grasp"] = bool(b1.ok and b2.ok)
    result["carry_steps"] = [bool(b0.ok), bool(b1.ok), bool(b2.ok)]
else:
    # ---- Strategy B: push from behind ----
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
    go(sx, sy, safe_z + 0.1, 0.04, 500)
    go(sx, sy, safe_z, 0.04, 400)
    r2 = go(sx, sy, push_z, 0.03, 400)
    r4 = go(ex, ey, push_z, 0.03, 500)
    go(ex, ey, safe_z, 0.05, 300)
    result["pushed"] = bool(r2.ok and r4.ok)
