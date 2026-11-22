base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "grasped": False, "rotated": False, "notes": []}


def locate(queries):
    for qq in queries:
        f = localize_object(qq, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 8:
            return f, qq
    return None, None


def P(x, y, z):
    return Pose((x, y, z), downward, base_frame)


def mv(x, y, z, tol=0.02):
    return move_to_pose(P(x, y, z), tolerance=tol, max_steps=250)


def qt(vals, f):
    s = sorted(vals)
    return s[int((len(s) - 1) * f)]


stove, sq = locate(("black flat stove", "stove", "flat stove burner"))
if stove is None:
    result["error"] = "stove not found"
else:
    result["localized"] = True
    pts = [(float(p[0]), float(p[1]), float(p[2])) for p in stove.point_cloud.points]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    top_z = qt(zs, 0.95)
    # knob: try its own prompt first
    knob, kq = locate(("stove knob", "black knob", "small knob"))
    kx = ky = kz = None
    if knob is not None:
        kp = [(float(p[0]), float(p[1]), float(p[2])) for p in knob.point_cloud.points]
        kxs = [p[0] for p in kp]
        kys = [p[1] for p in kp]
        if max(kxs) - min(kxs) < 0.12 and max(kys) - min(kys) < 0.12:
            kx, ky = qt(kxs, 0.5), qt(kys, 0.5)
            kz = qt([p[2] for p in kp], 0.9)
            result["knob_prompt"] = kq
    if kx is None:
        # fallback: highest points of stove cloud (knob protrudes)
        thr = qt(zs, 0.97)
        hi = [p for p in pts if p[2] >= thr]
        kx = sum(p[0] for p in hi) / len(hi)
        ky = sum(p[1] for p in hi) / len(hi)
        kz = thr
        result["notes"].append("fallback highest points")
    result["knob"] = [kx, ky, kz]
    open_gripper()
    r1 = mv(kx, ky, kz + 0.12)
    result["hover"] = r1.ok
    if r1.ok:
        r2 = mv(kx, ky, max(kz - 0.005, 0.0) + 0.005, 0.015)
        result["descend"] = r2.ok
        if r2.ok:
            close_gripper()
            result["grasped"] = True
            ik = solve_ik(P(kx, ky, kz + 0.005))
            if ik.ok:
                base = [float(v) for v in ik.joint_positions]
                for delta in (1.2, -2.4):
                    j = list(base)
                    j[-1] = j[-1] + delta
                    rr = move_to_joints(j, tolerance=0.03, max_steps=150)
                    result["rot_%s" % str(delta)] = rr.ok
                    result["rotated"] = True
            open_gripper()
            mv(kx, ky, kz + 0.12)
