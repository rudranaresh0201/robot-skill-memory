def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]

result = {"localized": False, "moved_above": False, "pressed": False}
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
    cx = quantile((p[0] for p in pts), 0.5)
    cy = quantile((p[1] for p in pts), 0.5)
    top_z = quantile((p[2] for p in pts), 0.9)
    # Knob is typically at the stove's front edge; use the geometry's extents-agnostic approach:
    # approach above the stove center first
    down = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
    close_gripper()
    hover = Pose((cx, cy, top_z + 0.15), down, base_frame)
    r = move_to_pose(hover, tolerance=0.02, max_steps=250)
    result["moved_above"] = bool(r.ok)
    if r.ok:
        # Press down on the stove's control knob/surface
        press = Pose((cx, cy, top_z + 0.02), down, base_frame)
        r2 = move_to_pose(press, tolerance=0.02, max_steps=250)
        result["pressed"] = bool(r2.ok)
        move_to_pose(hover, tolerance=0.03, max_steps=250)
