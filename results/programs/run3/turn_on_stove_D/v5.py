result = {"localized": False, "moved": False, "pushed": False, "attempts": []}

base_frame = get_robot_state().base_frame
quat = get_robot_state().end_effector_poses["primary"].quaternion_wxyz

def loc(queries):
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 15:
            return f
    return None

def med(v):
    s = sorted(float(x) for x in v)
    return s[len(s) // 2]

stove = loc(("flat stove", "black stove", "stove burner"))
if stove is None:
    result["error"] = "stove not found"
else:
    result["localized"] = True
    pts = stove.point_cloud.points
    xs = sorted(float(p[0]) for p in pts)
    cy = med(p[1] for p in pts)
    near_x = xs[int(len(xs) * 0.03)]
    kx, ky, ktop = near_x - 0.06, cy - 0.05, 0.06
    knob = loc(("black knob", "stove knob", "black cylindrical knob"))
    if knob is not None:
        kp = knob.point_cloud.points
        cx, cyk = med(p[0] for p in kp), med(p[1] for p in kp)
        ext = knob.geometry.extents
        if abs(cx - near_x) < 0.2 and abs(cyk - cy) < 0.2 and max(float(e) for e in ext) < 0.15:
            kx, ky = cx, cyk
            ktop = max(float(p[2]) for p in kp)
            result["knob_found"] = True
    result["knob"] = {"x": kx, "y": ky, "top": ktop}

    close_gripper()
    hover_z = ktop + 0.12
    low_z = max(ktop - 0.01, 0.03)
    half = 0.06
    plans = ((0.03, 1), (-0.03, 1), (0.03, -1), (-0.03, -1))
    for off, direc in plans:
        px = kx + off
        y0 = ky - direc * half
        y1 = ky + direc * half
        r = move_to_pose(Pose((px, y0, hover_z), quat, base_frame), tolerance=0.02, max_steps=200)
        if not r.ok:
            result["attempts"].append("hover_fail")
            continue
        result["moved"] = True
        r2 = move_to_pose(Pose((px, y0, low_z), quat, base_frame), tolerance=0.02, max_steps=150)
        if not r2.ok:
            result["attempts"].append("lower_fail")
            move_to_pose(Pose((px, y0, hover_z), quat, base_frame), tolerance=0.03, max_steps=150)
            continue
        r3 = move_to_pose(Pose((px, y1, low_z), quat, base_frame), tolerance=0.03, max_steps=150)
        result["attempts"].append({"off": off, "dir": direc, "ok": r3.ok})
        cur = get_robot_state().end_effector_poses["primary"].position
        move_to_pose(Pose((float(cur[0]), float(cur[1]), hover_z), quat, base_frame), tolerance=0.03, max_steps=150)
        if r3.ok:
            result["pushed"] = True
