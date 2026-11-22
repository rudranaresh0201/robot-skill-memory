def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]

base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "knob_found": False, "swept": False, "task": get_task_context().language}

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
xmin = quantile((p[0] for p in pts), 0.03)
ymin = quantile((p[1] for p in pts), 0.03)
ymax = quantile((p[1] for p in pts), 0.97)
result["stove_box"] = [float(xmin), float(ymin), float(ymax), float(top_z)]

# Knob candidate (validated against stove cloud)
kc = None
knob = locate(("stove knob", "knob", "black knob"))
if knob is not None:
    kp = knob.geometry.pose.position
    if abs(kp[0] - sc[0]) < 0.15 and abs(kp[1] - sc[1]) < 0.15:
        kc = kp
        result["knob_found"] = True
if kc is None:
    kc = (xmin + 0.03, sc[1], top_z)
result["knob_center"] = [float(v) for v in kc]

work_z = max(top_z, float(kc[2])) + 0.02
hover = work_z + 0.12
close_gripper()

def sweep(x, y_from, y_to):
    ok1 = move_to_pose(Pose((x, y_from, hover), downward, base_frame), tolerance=0.03, max_steps=250).ok
    ok2 = move_to_pose(Pose((x, y_from, work_z), downward, base_frame), tolerance=0.015, max_steps=250).ok
    ok3 = move_to_pose(Pose((x, y_to, work_z), downward, base_frame), tolerance=0.02, max_steps=250).ok
    move_to_pose(Pose((x, y_to, hover), downward, base_frame), tolerance=0.03, max_steps=200)
    return ok1 and ok2 and ok3

# Sweep across the knob region left-to-right, then a second pass slightly further in
done = sweep(float(kc[0]), float(kc[1]) - 0.08, float(kc[1]) + 0.08)
done2 = sweep(float(kc[0]) + 0.04, float(kc[1]) + 0.08, float(kc[1]) - 0.08)
result["swept"] = bool(done or done2)
