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

push_z = zmin + 0.025
safe_z = zmin + 0.15
sx = pc[0] - ux * (radius + 0.05)
sy = pc[1] - uy * (radius + 0.05)
ex = tx - ux * (radius + 0.02)
ey = ty - uy * (radius + 0.02)

close_gripper()
r1 = move_to_pose(Pose((sx, sy, safe_z), downward, base_frame), tolerance=0.02, max_steps=300)
r2 = move_to_pose(Pose((sx, sy, push_z), downward, base_frame), tolerance=0.015, max_steps=250)
mx, my = (sx + ex) / 2.0, (sy + ey) / 2.0
r3 = move_to_pose(Pose((mx, my, push_z), downward, base_frame), tolerance=0.02, max_steps=250)
r4 = move_to_pose(Pose((ex, ey, push_z), downward, base_frame), tolerance=0.02, max_steps=250)
move_to_pose(Pose((ex, ey, safe_z), downward, base_frame), tolerance=0.03, max_steps=200)

result["pushed"] = bool(r4.ok)
result["steps_ok"] = [bool(r1.ok), bool(r2.ok), bool(r3.ok), bool(r4.ok)]
result["target"] = (tx, ty)
