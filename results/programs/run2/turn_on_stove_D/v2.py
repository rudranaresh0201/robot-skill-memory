result = {"localized": False, "moved": False, "notes": ""}

base_frame = get_robot_state().base_frame
found = None
for q in ("flat stove", "black stove", "stove burner"):
    f = localize_object(q, camera_name="agentview", target_frame=base_frame)
    if f.ok and len(f.point_cloud.points) >= 20:
        found = f
        break

if found is None:
    result["notes"] = "stove not found"
else:
    result["localized"] = True
    pts = found.point_cloud.points
    xs = sorted(float(p[0]) for p in pts)
    ys = sorted(float(p[1]) for p in pts)
    zs = sorted(float(p[2]) for p in pts)
    n = len(xs)
    xmin = xs[int((n - 1) * 0.02)]
    xmax = xs[int((n - 1) * 0.98)]
    ymin = ys[int((n - 1) * 0.02)]
    ymax = ys[int((n - 1) * 0.98)]
    top_z = zs[int((n - 1) * 0.9)]
    quat = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
    result["bounds"] = [xmin, xmax, ymin, ymax, top_z]
    close_gripper()
    attempts = 0
    try:
        z_low = top_z + 0.03
        z_hi = top_z + 0.15
        for ex in (xmax - 0.03, xmax + 0.01, xmin + 0.03):
            for (ya, yb) in ((ymin - 0.04, ymax + 0.04), (ymax + 0.04, ymin - 0.04)):
                attempts += 1
                move_to_pose(Pose((ex, ya, z_hi), quat, base_frame), tolerance=0.02, max_steps=250)
                move_to_pose(Pose((ex, ya, z_low), quat, base_frame), tolerance=0.02, max_steps=250)
                move_to_pose(Pose((ex, (ya + yb) / 2.0, z_low), quat, base_frame), tolerance=0.02, max_steps=250)
                move_to_pose(Pose((ex, yb, z_low), quat, base_frame), tolerance=0.02, max_steps=250)
                move_to_pose(Pose((ex, yb, z_hi), quat, base_frame), tolerance=0.02, max_steps=250)
        result["moved"] = True
    except Exception as e:
        result["notes"] = "motion error: " + str(e)
    result["attempts"] = attempts
