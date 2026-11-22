result = {"localized": False, "moved": False, "pushed": False}

base_frame = get_robot_state().base_frame
quat = get_robot_state().end_effector_poses["primary"].quaternion_wxyz

found = None
for q in ("flat stove", "black stove", "stove burner"):
    f = localize_object(q, camera_name="agentview", target_frame=base_frame)
    if f.ok and len(f.point_cloud.points) >= 20:
        found = f
        break

if found is None:
    result["error"] = "stove not found"
else:
    result["localized"] = True
    pts = found.point_cloud.points
    xs = sorted(float(p[0]) for p in pts)
    ys = sorted(float(p[1]) for p in pts)
    zs = sorted(float(p[2]) for p in pts)
    cx = xs[len(xs) // 2]
    cy = ys[len(ys) // 2]
    top = zs[int(len(zs) * 0.95)]
    front_x = xs[int(len(xs) * 0.05)]
    right_y = ys[int(len(ys) * 0.95)]
    left_y = ys[int(len(ys) * 0.05)]
    result["stove"] = {"cx": cx, "cy": cy, "top": top, "front_x": front_x}

    close_gripper()
    hover_z = top + 0.12
    r = move_to_pose(Pose((front_x + 0.02, right_y - 0.03, hover_z), quat, base_frame),
                     tolerance=0.02, max_steps=250)
    result["moved"] = r.ok
    if r.ok:
        r2 = move_to_pose(Pose((front_x + 0.02, right_y - 0.03, top + 0.03), quat, base_frame),
                          tolerance=0.02, max_steps=200)
        if r2.ok:
            r3 = move_to_pose(Pose((front_x + 0.02, right_y - 0.12, top + 0.03), quat, base_frame),
                              tolerance=0.02, max_steps=200)
            result["pushed"] = r3.ok
            if not r3.ok:
                move_to_pose(Pose((front_x + 0.02, left_y + 0.03, top + 0.03), quat, base_frame),
                             tolerance=0.03, max_steps=200)
        move_to_pose(Pose((front_x + 0.02, right_y - 0.05, hover_z), quat, base_frame),
                     tolerance=0.03, max_steps=200)
