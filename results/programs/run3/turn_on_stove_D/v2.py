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
    cy = ys[len(ys) // 2]
    top = zs[int(len(zs) * 0.95)]
    far_x = xs[int(len(xs) * 0.97)]
    result["stove"] = {"cy": cy, "top": top, "far_x": far_x}

    close_gripper()
    # knob sits on the far (camera-facing) edge of the stove
    kx = far_x - 0.025
    hover_z = top + 0.12
    low_z = top + 0.03
    y0 = cy - 0.07
    y1 = cy + 0.06
    r = move_to_pose(Pose((kx, y0, hover_z), quat, base_frame), tolerance=0.02, max_steps=250)
    result["moved"] = r.ok
    if r.ok:
        r2 = move_to_pose(Pose((kx, y0, low_z), quat, base_frame), tolerance=0.02, max_steps=200)
        result["lowered"] = r2.ok
        if r2.ok:
            r3 = move_to_pose(Pose((kx, y1, low_z), quat, base_frame), tolerance=0.02, max_steps=250)
            result["pushed"] = r3.ok
            move_to_pose(Pose((kx, y1, hover_z), quat, base_frame), tolerance=0.03, max_steps=200)
