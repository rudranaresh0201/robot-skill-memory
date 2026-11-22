def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]

result = {"localized": False, "moved_above": False, "swept": False}
base_frame = get_robot_state().base_frame

found = None
for q in ("flat stove", "black stove", "stove burner", "stove"):
    f = localize_object(q, camera_name="agentview", target_frame=base_frame)
    if f.ok and len(f.point_cloud.points) >= 20:
        found = f
        break

if found is None:
    result["error"] = "stove not found"
else:
    result["localized"] = True
    pts = found.point_cloud.points
    xmax = quantile((p[0] for p in pts), 0.97)
    xmin = quantile((p[0] for p in pts), 0.03)
    ymin = quantile((p[1] for p in pts), 0.03)
    ymax = quantile((p[1] for p in pts), 0.97)
    surf_z = quantile((p[2] for p in pts), 0.5)
    down = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
    close_gripper()
    low_z = surf_z + 0.03
    hover_z = surf_z + 0.15
    # Knob is near the front (large x) left (small y) corner of the stove.
    # Try several front-edge x offsets, sweeping sideways along +y at low height.
    attempts = [xmax - 0.01, xmax + 0.02, xmax - 0.04]
    start_y = ymin - 0.04
    count = 0
    for ax in attempts:
        r = move_to_pose(Pose((ax, start_y, hover_z), down, base_frame), tolerance=0.02, max_steps=250)
        if not r.ok:
            continue
        result["moved_above"] = True
        r = move_to_pose(Pose((ax, start_y, low_z), down, base_frame), tolerance=0.02, max_steps=250)
        if not r.ok:
            continue
        for k in range(1, 5):
            yy = start_y + 0.04 * k
            rr = move_to_pose(Pose((ax, yy, low_z), down, base_frame), tolerance=0.02, max_steps=150)
            if not rr.ok:
                break
            count += 1
        move_to_pose(Pose((ax, start_y + 0.16, hover_z), down, base_frame), tolerance=0.03, max_steps=250)
    result["swept"] = count > 0
    result["sweep_steps"] = count
