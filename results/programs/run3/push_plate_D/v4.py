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
radius = min(max(max(xs) - min(xs), max(ys) - min(ys)) / 2.0, 0.13)
cx = sum(xs) / len(xs)
cy = sum(ys) / len(ys)
push_z = min(zs) + 0.03

stove = locate(("black stove", "flat stove", "stove burner", "stove"))
if stove is not None:
    ty = stove.geometry.pose.position[1]
    result["stove_y"] = ty
else:
    ty = cy + 0.25
if ty - cy < 0.1:
    ty = cy + 0.25
# front of stove is toward the camera (+x in base frame)
tx = cx + 0.12

dx, dy = tx - cx, ty - cy
dist = math.sqrt(dx * dx + dy * dy)
ux, uy = dx / dist, dy / dist
gap = radius + 0.025
sx, sy = cx - ux * gap, cy - uy * gap
result["start"] = (sx, sy)
result["end"] = (tx, ty)

close_gripper()
hi = push_z + 0.15
r = move_to_pose(Pose((sx, sy, hi), down, base_frame), tolerance=0.03, max_steps=250)
result["above_start"] = r.ok
if r.ok:
    r = move_to_pose(Pose((sx, sy, push_z), down, base_frame), tolerance=0.03, max_steps=200)
    result["lowered"] = r.ok
    if not r.ok:
        # fallback: start from directly behind the plate (robot side), push toward +x then +y
        bx = cx - gap - 0.02
        r1 = move_to_pose(Pose((sx, sy, hi + 0.05), down, base_frame), tolerance=0.04, max_steps=150)
        r2 = move_to_pose(Pose((bx, cy, hi + 0.05), down, base_frame), tolerance=0.03, max_steps=200)
        if r2.ok:
            r3 = move_to_pose(Pose((bx, cy, push_z), down, base_frame), tolerance=0.03, max_steps=200)
            result["fallback_lowered"] = r3.ok
            if r3.ok:
                r4 = move_to_pose(Pose((tx - gap, cy, push_z), down, base_frame), tolerance=0.03, max_steps=300)
                result["pushed"] = r4.ok
                move_to_pose(Pose((tx - gap, cy, push_z + 0.15), down, base_frame), tolerance=0.04, max_steps=150)
    else:
        r = move_to_pose(Pose((tx, ty, push_z), down, base_frame), tolerance=0.03, max_steps=400)
        result["pushed"] = r.ok
        r2 = move_to_pose(Pose((tx, ty, push_z + 0.15), down, base_frame), tolerance=0.04, max_steps=200)
        result["retreated"] = r2.ok
