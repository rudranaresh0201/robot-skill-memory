def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]

base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "knob_found": False, "sweeps": [], "task": get_task_context().language}

def locate(queries):
    for q in queries:
        found = localize_object(q, camera_name="agentview", target_frame=base_frame)
        if found.ok and len(found.point_cloud.points) >= 20:
            return found
    return None

stove = locate(("black flat stove", "stove", "stove burner"))
if stove is None:
    raise ValueError("stove not found")
result["localized"] = True
pts = stove.point_cloud.points
sc = stove.geometry.pose.position
top_z = quantile((p[2] for p in pts), 0.95)
ymin = quantile((p[1] for p in pts), 0.03)

kc = None
knob = locate(("stove knob", "knob", "black knob"))
if knob is not None:
    kp = knob.geometry.pose.position
    if abs(kp[0] - sc[0]) < 0.2 and abs(kp[1] - sc[1]) < 0.2:
        kc = kp
        result["knob_found"] = True
if kc is None:
    kc = (sc[0], ymin, top_z)
kx = float(kc[0])
ky = float(kc[1])
kz = float(kc[2])
result["knob_center"] = [kx, ky, kz]

hover = kz + 0.09
close_gripper()

def pt(x, y, z):
    return Pose((x, y, z), downward, base_frame)

def sweep(x0, y0, x1, y1, z):
    if not move_to_pose(pt(x0, y0, hover), tolerance=0.03, max_steps=250).ok:
        return False
    if not move_to_pose(pt(x0, y0, z), tolerance=0.025, max_steps=200).ok:
        move_to_pose(pt(x0, y0, hover), tolerance=0.03, max_steps=150)
        return False
    ok = move_to_pose(pt(x1, y1, z), tolerance=0.025, max_steps=250).ok
    move_to_pose(pt(x1, y1, hover), tolerance=0.03, max_steps=200)
    return ok

low_z = kz + 0.015
plans = [
    (kx + 0.015, ky - 0.06, kx + 0.015, ky + 0.06, low_z),
    (kx - 0.015, ky + 0.06, kx - 0.015, ky - 0.06, low_z),
    (kx - 0.05, ky + 0.015, kx + 0.03, ky + 0.015, low_z),
    (kx + 0.03, ky - 0.015, kx - 0.05, ky - 0.015, low_z),
]
for p in plans:
    result["sweeps"].append(bool(sweep(*p)))
