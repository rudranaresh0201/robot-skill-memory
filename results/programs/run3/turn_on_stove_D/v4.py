result = {"localized": False, "moved": False, "pushed": False}

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
    # default guess: knob sits between robot and stove, slightly to one side
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
    low_z = max(ktop - 0.015, 0.03)
    d = 0.015
    r = move_to_pose(Pose((kx + d, ky - 0.07, hover_z), quat, base_frame), tolerance=0.02, max_steps=250)
    result["moved"] = r.ok
    if r.ok:
        r2 = move_to_pose(Pose((kx + d, ky - 0.07, low_z), quat, base_frame), tolerance=0.02, max_steps=200)
        result["lowered"] = r2.ok
        if r2.ok:
            r3 = move_to_pose(Pose((kx + d, ky + 0.07, low_z), quat, base_frame), tolerance=0.02, max_steps=250)
            result["pushed"] = r3.ok
            if r3.ok:
                r4 = move_to_pose(Pose((kx - d, ky + 0.07, low_z), quat, base_frame), tolerance=0.02, max_steps=150)
                if r4.ok:
                    r5 = move_to_pose(Pose((kx - d, ky - 0.07, low_z), quat, base_frame), tolerance=0.02, max_steps=250)
                    result["pushed2"] = r5.ok
            move_to_pose(Pose((kx, ky, hover_z), quat, base_frame), tolerance=0.03, max_steps=200)
