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

def go(x, y, z, quat, tol=0.03, steps=250):
    r = move_to_pose(Pose((x, y, z), quat, base_frame), tolerance=tol, max_steps=steps)
    return r.ok

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
    hover_z = surf_z + 0.15
    xc = 0.5 * (xmin + xmax)
    rows = [xmax - 0.03, xc + 0.5 * (xmax - xc), xc]
    count = 0
    reached = []
    for ax in rows:
        ys = [ymin + (ymax - ymin) * k / 4.0 for k in range(5)]
        if not go(ax, ys[0], hover_z, down):
            continue
        result["moved_above"] = True
        low = None
        for dz in (0.02, 0.04, 0.06, 0.08):
            if go(ax, ys[0], surf_z + dz, down, tol=0.03, steps=200):
                low = surf_z + dz
                break
        if low is None:
            continue
        reached.append(low - surf_z)
        for yy in ys[1:]:
            if not go(ax, yy, low, down, tol=0.03, steps=150):
                break
            count += 1
        # back and forth to catch knob either direction
        for yy in reversed(ys[:-1]):
            if not go(ax, yy, low, down, tol=0.03, steps=150):
                break
            count += 1
        go(ax, ys[0], hover_z, down)
    result["swept"] = count > 0
    result["sweep_steps"] = count
    result["low_offsets"] = reached
