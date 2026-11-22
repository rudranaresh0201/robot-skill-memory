def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]

result = {"localized": False, "moved": False, "grasped": False, "rotated": False}
base_frame = get_robot_state().base_frame
down = get_robot_state().end_effector_poses["primary"].quaternion_wxyz

knob = None
for q in ("black knob", "stove knob", "black knob on the stove", "black round knob"):
    k = localize_object(q, camera_name="agentview", target_frame=base_frame)
    if k.ok and len(k.point_cloud.points) >= 10:
        knob = k
        break

if knob is None:
    result["error"] = "knob not found"
else:
    result["localized"] = True
    kp = knob.point_cloud.points
    kx = quantile((p[0] for p in kp), 0.5)
    ky = quantile((p[1] for p in kp), 0.5)
    kz = quantile((p[2] for p in kp), 0.9)
    result["knob_xyz"] = [kx, ky, kz]

    open_gripper()
    r = move_to_pose(Pose((kx, ky, kz + 0.12), down, base_frame), tolerance=0.02, max_steps=250)
    result["moved"] = r.ok
    if r.ok:
        r2 = move_to_pose(Pose((kx, ky, kz - 0.01), down, base_frame), tolerance=0.015, max_steps=250)
        result["lowered"] = r2.ok
        if r2.ok:
            close_gripper()
            result["grasped"] = True
            q = list(get_robot_state().joint_positions["primary"])
            q[6] = q[6] + 0.9
            r3 = move_to_joints(q, tolerance=0.03, max_steps=200)
            result["rotated"] = r3.ok
            open_gripper()
            move_to_pose(Pose((kx, ky, kz + 0.12), down, base_frame), tolerance=0.03, max_steps=150)
