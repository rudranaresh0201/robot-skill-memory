base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "moved": False}

def locate(queries):
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None

def mv(x, y, z, tol=0.02):
    return move_to_pose(Pose((x, y, z), downward, base_frame), tolerance=tol, max_steps=250)

stove = locate(("black flat stove", "stove", "flat stove burner"))
if stove is None:
    result["error"] = "stove not found"
else:
    result["localized"] = True
    pts = [(float(p[0]), float(p[1]), float(p[2])) for p in stove.point_cloud.points]
    xs = sorted(p[0] for p in pts)
    ys = sorted(p[1] for p in pts)
    zs = sorted(p[2] for p in pts)
    xmax = xs[int((len(xs) - 1) * 0.97)]
    ymin = ys[int((len(ys) - 1) * 0.03)]
    top_z = zs[int((len(zs) - 1) * 0.95)]
    # Knob sits at the front-left corner of the stove (larger x, smaller y)
    kx = xmax - 0.03
    ky = ymin + 0.03
    near = [p[2] for p in pts if abs(p[0] - kx) < 0.04 and abs(p[1] - ky) < 0.04]
    kz = max(near) if near else top_z
    result["knob_guess"] = [kx, ky, kz]
    close_gripper()
    r1 = mv(kx, ky - 0.04, kz + 0.12)
    result["hover_ok"] = r1.ok
    if r1.ok:
        r2 = mv(kx, ky - 0.04, kz + 0.015, 0.015)
        result["descend_ok"] = r2.ok
        if r2.ok:
            # sweep sideways through the knob to rotate it
            r3 = mv(kx, ky + 0.05, kz + 0.015, 0.015)
            result["sweep1_ok"] = r3.ok
            r4 = mv(kx, ky + 0.05, kz + 0.12)
            result["moved"] = r3.ok
            # second sweep along x direction from the other side
            r5 = mv(kx + 0.04, ky, kz + 0.12)
            if r5.ok:
                r6 = mv(kx + 0.04, ky, kz + 0.015, 0.015)
                if r6.ok:
                    r7 = mv(kx - 0.04, ky, kz + 0.015, 0.015)
                    result["sweep2_ok"] = r7.ok
                mv(kx, ky, kz + 0.12)
