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


def mv(x, y, z, tol=0.02, steps=200):
    return move_to_pose(P(x, y, z), tolerance=tol, max_steps=steps)


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
    cands = []
    knob, kq = locate(("stove knob", "black knob", "small knob"))
    if knob is not None:
        kp = [(float(p[0]), float(p[1]), float(p[2])) for p in knob.point_cloud.points]
        kxs = [p[0] for p in kp]
        kys = [p[1] for p in kp]
        if max(kxs) - min(kxs) < 0.12 and max(kys) - min(kys) < 0.12:
            cands.append((qt(kxs, 0.5), qt(kys, 0.5), qt([p[2] for p in kp], 0.9), "knob"))
    thr = qt(zs, 0.9)
    hi = [p for p in pts if p[2] >= thr]
    cands.append((sum(p[0] for p in hi) / len(hi), sum(p[1] for p in hi) / len(hi), thr, "high"))
    cands.append((qt(xs, 0.5), qt(ys, 0.5), qt(zs, 0.9), "center"))
    result["cands"] = [list(c[:3]) + [c[3]] for c in cands]

    open_gripper()
    chosen = None
    for kx, ky, kz, nm in cands:
        r1 = mv(kx, ky, kz + 0.10)
        result["hover_" + nm] = r1.ok
        if not r1.ok:
            continue
        r2 = mv(kx, ky, kz + 0.005, 0.015)
        result["descend_" + nm] = r2.ok
        if r2.ok:
            chosen = (kx, ky, kz)
            break
        mv(kx, ky, kz + 0.10)
    if chosen is not None:
        kx, ky, kz = chosen
        close_gripper()
        result["grasped"] = True
        st = get_robot_state()
        base = [float(v) for v in st.joint_positions["primary"]]
        for delta in (0.9, -1.8):
            j = list(base)
            j[-1] = j[-1] + delta
            rr = move_to_joints(j, tolerance=0.05, max_steps=300)
            result["rot_%s" % str(delta)] = rr.ok
            result["rotated"] = True
        open_gripper()
        mv(kx, ky, kz + 0.10)
