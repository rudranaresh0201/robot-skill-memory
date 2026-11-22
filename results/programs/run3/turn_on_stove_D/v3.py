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
    near_x = xs[int(len(xs) * 0.03)]
    result["stove"] = {"cy": cy, "top": top, "near_x": near_x}

    close_gripper()
    # The knob is on the robot-facing (near) edge of the stove, within reach.
    hover_z = top + 0.14
    low_z = top + 0.04
    y0 = cy - 0.08
    y1 = cy + 0.06
    done = False
    for off in (0.02, 0.05, 0.08):
        kx = near_x + off
        r = move_to_pose(Pose((kx, y0, hover_z), quat, base_frame), tolerance=0.02, max_steps=250)
        result["moved"] = r.ok
        if not r.ok:
            continue
        r2 = move_to_pose(Pose((kx, y0, low_z), quat, base_frame), tolerance=0.02, max_steps=200)
        result["lowered"] = r2.ok
        if not r2.ok:
            continue
        r3 = move_to_pose(Pose((kx, y1, low_z), quat, base_frame), tolerance=0.02, max_steps=250)
        result["pushed"] = r3.ok
        result["kx"] = kx
        move_to_pose(Pose((kx, y1, hover_z), quat, base_frame), tolerance=0.03, max_steps=200)
        done = True
        break
    result["done"] = done
