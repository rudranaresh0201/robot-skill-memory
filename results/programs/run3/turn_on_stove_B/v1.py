def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]

result = {"localized": False, "moved": False, "swept": False}
base_frame = get_robot_state().base_frame
down = get_robot_state().end_effector_poses["primary"].quaternion_wxyz

found = None
knob = None
for q in ("black knob on the stove", "stove knob"):
    k = localize_object(q, camera_name="agentview", target_frame=base_frame)
    if k.ok and len(k.point_cloud.points) >= 10:
        knob = k
        break
for q in ("flat stove", "black stove", "stove"):
    f = localize_object(q, camera_name="agentview", target_frame=base_frame)
    if f.ok and len(f.point_cloud.points) >= 20:
        found = f
        break

if found is None:
    result["error"] = "stove not found"
else:
    result["localized"] = True
    pts = found.point_cloud.points
    cx = quantile((p[0] for p in pts), 0.5)
    cy = quantile((p[1] for p in pts), 0.5)
    minx = quantile((p[0] for p in pts), 0.05)
    top = quantile((p[2] for p in pts), 0.9)
    if knob is not None:
        kp = knob.point_cloud.points
        kx = quantile((p[0] for p in kp), 0.5)
        ky = quantile((p[1] for p in kp), 0.5)
        kz = quantile((p[2] for p in kp), 0.9)
    else:
        kx, ky, kz = minx + 0.03, cy, top
    result["knob_used"] = knob is not None

    close_gripper()
    hover = Pose((kx - 0.03, ky - 0.08, kz + 0.12), down, base_frame)
    r = move_to_pose(hover, tolerance=0.02, max_steps=250)
    result["moved"] = r.ok
    if r.ok:
        low = Pose((kx - 0.03, ky - 0.08, kz + 0.01), down, base_frame)
        r2 = move_to_pose(low, tolerance=0.02, max_steps=250)
        result["lowered"] = r2.ok
        if r2.ok:
            ok_all = True
            for dy in (-0.04, 0.0, 0.04, 0.08):
                rs = move_to_pose(Pose((kx - 0.03, ky + dy, kz + 0.01), down, base_frame),
                                  tolerance=0.02, max_steps=150)
                ok_all = ok_all and rs.ok
                if not rs.ok:
                    break
            result["swept"] = ok_all
            move_to_pose(Pose((kx - 0.03, ky + 0.08, kz + 0.12), down, base_frame),
                         tolerance=0.03, max_steps=150)
