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
    cx = found.geometry.pose.position[0]
    cy = found.geometry.pose.position[1]
    top_z = zs[int((len(zs) - 1) * 0.95)]
    front_x = xs[int((len(xs) - 1) * 0.05)]
    quat = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
    result["stove_center"] = [float(cx), float(cy), float(top_z)]

    close_gripper()
    try:
        # hover in front of stove near the knob (front edge), then descend
        hover = Pose((front_x - 0.02, cy, top_z + 0.12), quat, base_frame)
        r1 = move_to_pose(hover, tolerance=0.02, max_steps=250)
        low = Pose((front_x - 0.02, cy, top_z + 0.03), quat, base_frame)
        r2 = move_to_pose(low, tolerance=0.02, max_steps=250)
        # sweep laterally to push/turn the knob, both directions
        for dy in (0.08, -0.08, 0.08):
            sweep = Pose((front_x - 0.02, cy + dy, top_z + 0.03), quat, base_frame)
            move_to_pose(sweep, tolerance=0.02, max_steps=250)
        # push forward onto the knob
        push = Pose((front_x + 0.03, cy, top_z + 0.03), quat, base_frame)
        move_to_pose(push, tolerance=0.02, max_steps=250)
        result["moved"] = True
    except Exception as e:
        result["notes"] = "motion error: " + str(e)
