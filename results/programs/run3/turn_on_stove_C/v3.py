base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "moved": False}


def locate(queries):
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None


def P(x, y, z):
    return Pose((x, y, z), downward, base_frame)


def reachable(x, y, z):
    return solve_ik(P(x, y, z)).ok


def mv(x, y, z, tol=0.02):
    return move_to_pose(P(x, y, z), tolerance=tol, max_steps=250)


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
    kx = xmax - 0.03
    ky = ymin + 0.03
    near = [p[2] for p in pts if abs(p[0] - kx) < 0.04 and abs(p[1] - ky) < 0.04]
    kz = max(near) if near else top_z
    result["knob_guess"] = [kx, ky, kz]
    close_gripper()

    # Pick candidate approach points that IK can actually reach.
    chosen = None
    for dx in (0.0, -0.04, -0.08, -0.12):
        for hz in (0.12, 0.18, 0.25):
            if reachable(kx + dx, ky - 0.04, kz + hz):
                chosen = (kx + dx, hz)
                break
        if chosen:
            break
    result["chosen"] = chosen
    if chosen is None:
        result["error"] = "no reachable hover"
    else:
        cx, hz = chosen
        r1 = mv(cx, ky - 0.04, kz + hz)
        result["hover_ok"] = r1.ok
        if r1.ok:
            r2 = mv(cx, ky - 0.04, kz + 0.02, 0.02)
            result["descend_ok"] = r2.ok
            if r2.ok:
                r3 = mv(cx, ky + 0.05, kz + 0.02, 0.02)
                result["sweep1_ok"] = r3.ok
                result["moved"] = r3.ok
                mv(cx, ky + 0.05, kz + hz)
                r5 = mv(cx + 0.04, ky, kz + hz)
                if r5.ok:
                    r6 = mv(cx + 0.04, ky, kz + 0.02, 0.02)
                    if r6.ok:
                        r7 = mv(cx - 0.04, ky, kz + 0.02, 0.02)
                        result["sweep2_ok"] = r7.ok
                    mv(cx, ky, kz + hz)
