base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "moved": False}

def locate(queries):
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None

stove = locate(("black flat stove", "stove", "flat stove burner"))
if stove is None:
    result["error"] = "stove not found"
else:
    result["localized"] = True
    pts = stove.point_cloud.points
    zs = sorted(float(p[2]) for p in pts)
    top_z = zs[int((len(zs) - 1) * 0.95)]
    c = stove.geometry.pose.position
    result["center"] = [float(v) for v in c]
    # Knob is typically at the front edge of the stove; probe by pressing/turning near it.
    xs = sorted(float(p[0]) for p in pts)
    ys = sorted(float(p[1]) for p in pts)
    cx = (xs[0] + xs[-1]) / 2.0
    cy = (ys[0] + ys[-1]) / 2.0
    close_gripper()
    hover = Pose((cx, cy, top_z + 0.12), downward, base_frame)
    r1 = move_to_pose(hover, tolerance=0.02, max_steps=250)
    result["hover_ok"] = r1.ok
    if r1.ok:
        # Press down on the stove's control surface to toggle the switch.
        r2 = move_to_pose(Pose((cx, cy, top_z + 0.02), downward, base_frame),
                          tolerance=0.02, max_steps=250)
        result["press_ok"] = r2.ok
        r3 = move_to_pose(hover, tolerance=0.02, max_steps=250)
        result["retreat_ok"] = r3.ok
        result["moved"] = r2.ok
