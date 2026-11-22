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
sc = stove.geometry.pose.position
pts = stove.point_cloud.points
top_z = quantile((p[2] for p in pts), 0.9)

knob = locate(("stove knob", "knob", "black knob"))
if knob is not None and abs(knob.geometry.pose.position[0] - sc[0]) < 0.25 and abs(knob.geometry.pose.position[1] - sc[1]) < 0.25:
    kc = knob.geometry.pose.position
    kz = quantile((p[2] for p in knob.point_cloud.points), 0.9)
    result["knob_found"] = True
else:
    # fallback: knob near the front edge (toward robot, smaller x) of stove
    kc = (quantile((p[0] for p in pts), 0.1), sc[1], top_z)
    kz = top_z
result["knob_center"] = [float(v) for v in kc]

close_gripper()
hover = kz + 0.10
move_to_pose(Pose((kc[0], kc[1] - 0.05, hover), downward, base_frame), tolerance=0.02, max_steps=250)
move_to_pose(Pose((kc[0], kc[1] - 0.05, kz + 0.015), downward, base_frame), tolerance=0.015, max_steps=250)
r1 = move_to_pose(Pose((kc[0], kc[1] + 0.0, kz + 0.015), downward, base_frame), tolerance=0.015, max_steps=200)
r2 = move_to_pose(Pose((kc[0], kc[1] + 0.05, kz + 0.015), downward, base_frame), tolerance=0.015, max_steps=200)
result["swept"] = bool(r1.ok or r2.ok)
move_to_pose(Pose((kc[0], kc[1] + 0.05, hover), downward, base_frame), tolerance=0.03, max_steps=200)
