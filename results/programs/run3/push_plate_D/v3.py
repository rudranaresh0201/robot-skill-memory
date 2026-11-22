import math

def locate(queries):
    base_frame = get_robot_state().base_frame
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None

state = get_robot_state()
base_frame = state.base_frame
down = state.end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "pushed": False}

plate = locate(("plate", "white plate", "round plate"))
if plate is None:
    raise ValueError("plate not found")
result["localized"] = True
pts = plate.point_cloud.points
zs = [p[2] for p in pts]
xs = [p[0] for p in pts]
ys = [p[1] for p in pts]
radius = max(max(xs) - min(xs), max(ys) - min(ys)) / 2.0
radius = min(radius, 0.13)
cx = sum(xs) / len(xs)
cy = sum(ys) / len(ys)
push_z = min(zs) + 0.025

stove = locate(("black stove", "flat stove", "stove burner", "stove"))
if stove is not None:
    sc = stove.geometry.pose.position
    ty = sc[1]
    result["stove_y"] = ty
else:
    ty = cy + 0.22
if ty - cy < 0.08:
    ty = cy + 0.22
tx = cx + 0.03

# push mostly along +y (image right), start close behind plate
dx, dy = tx - cx, ty - cy
dist = math.sqrt(dx * dx + dy * dy)
ux, uy = dx / dist, dy / dist
gap = radius + 0.03
sx, sy = cx - ux * gap, cy - uy * gap
ex, ey = tx - ux * 0.0, ty - uy * 0.0
result["start"] = (sx, sy)
result["end"] = (ex, ey)

close_gripper()
hi = push_z + 0.22
r = move_to_pose(Pose((sx, sy, hi), down, base_frame), tolerance=0.03, max_steps=250)
result["above_start"] = r.ok
if r.ok:
    r = move_to_pose(Pose((sx, sy, push_z), down, base_frame), tolerance=0.02, max_steps=250)
    result["lowered"] = r.ok
    if r.ok:
        r = move_to_pose(Pose((ex, ey, push_z), down, base_frame), tolerance=0.03, max_steps=400)
        result["pushed"] = r.ok
        r2 = move_to_pose(Pose((ex, ey, push_z + 0.15), down, base_frame), tolerance=0.04, max_steps=200)
        result["retreated"] = r2.ok
