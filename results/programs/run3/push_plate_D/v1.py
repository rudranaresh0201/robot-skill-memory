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
c = plate.geometry.pose.position
zs = [p[2] for p in plate.point_cloud.points]
xs = [p[0] for p in plate.point_cloud.points]
ys = [p[1] for p in plate.point_cloud.points]
radius = max(max(xs) - min(xs), max(ys) - min(ys)) / 2.0
cx = sum(xs) / len(xs)
cy = sum(ys) / len(ys)
push_z = min(zs) + 0.015

# Table-frame offset from plate start to stove front region
dx, dy = 0.15, 0.26
dist = math.sqrt(dx * dx + dy * dy)
ux, uy = dx / dist, dy / dist
gap = radius + 0.05
sx, sy = cx - ux * gap, cy - uy * gap
tx, ty = cx + dx - ux * (radius * 0.0) - ux * (gap - gap), cy + dy
# arm end: plate center target minus contact offset
ex, ey = tx - ux * (radius + 0.01), ty - uy * (radius + 0.01)

close_gripper()
r = move_to_pose(Pose((sx, sy, push_z + 0.12), down, base_frame), tolerance=0.02, max_steps=250)
result["above_start"] = r.ok
if r.ok:
    r = move_to_pose(Pose((sx, sy, push_z), down, base_frame), tolerance=0.015, max_steps=250)
    result["lowered"] = r.ok
    if r.ok:
        r = move_to_pose(Pose((ex, ey, push_z), down, base_frame), tolerance=0.02, max_steps=400)
        result["pushed"] = r.ok
        move_to_pose(Pose((ex, ey, push_z + 0.12), down, base_frame), tolerance=0.03, max_steps=200)
