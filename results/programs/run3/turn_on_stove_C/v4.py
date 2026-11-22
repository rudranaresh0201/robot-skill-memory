base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "moved": False, "attempts": []}


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


def q(vals, f):
    s = sorted(vals)
    return s[int((len(s) - 1) * f)]


stove = locate(("black flat stove", "stove", "flat stove burner"))
if stove is None:
    result["error"] = "stove not found"
else:
    result["localized"] = True
    pts = [(float(p[0]), float(p[1]), float(p[2])) for p in stove.point_cloud.points]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    xmin, xmax = q(xs, 0.03), q(xs, 0.97)
    ymin, ymax = q(ys, 0.03), q(ys, 0.97)
    cx, cy = q(xs, 0.5), q(ys, 0.5)
    top_z = q(zs, 0.95)
    result["stove"] = [cx, cy, top_z, xmin, xmax, ymin, ymax]
    close_gripper()

    # candidate edge points with tangent sweep direction
    cands = [
        (xmax - 0.03, cy, 0.0, 1.0),
        (xmin + 0.03, cy, 0.0, 1.0),
        (cx, ymax - 0.03, 1.0, 0.0),
        (cx, ymin + 0.03, 1.0, 0.0),
    ]
    # order by nearest to robot (smaller reach first)
    cands = sorted(cands, key=lambda c: c[0] * c[0] + c[1] * c[1])
    hz = 0.12
    tries = 0
    for (kx, ky, tx, ty) in cands:
        if tries >= 3 or result["moved"]:
            break
        a = (kx - 0.05 * tx, ky - 0.05 * ty)
        b = (kx + 0.05 * tx, ky + 0.05 * ty)
        lowz = top_z + 0.03
        ok = (reachable(kx, ky, top_z + hz) and reachable(a[0], a[1], lowz)
              and reachable(b[0], b[1], lowz))
        if not ok:
            result["attempts"].append({"c": [kx, ky], "reachable": False})
            continue
        tries += 1
        r1 = mv(a[0], a[1], top_z + hz)
        info = {"c": [kx, ky], "hover": r1.ok}
        if r1.ok:
            r2 = mv(a[0], a[1], lowz, 0.02)
            info["descend"] = r2.ok
            if r2.ok:
                r3 = mv(b[0], b[1], lowz, 0.02)
                info["sweep"] = r3.ok
                result["moved"] = result["moved"] or r3.ok
                mv(b[0], b[1], top_z + hz)
        result["attempts"].append(info)
        if not r1.ok:
            break
