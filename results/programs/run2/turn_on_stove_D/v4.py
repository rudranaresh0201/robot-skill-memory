result = {"localized": False, "knob_found": False, "moved": False, "notes": ""}

base_frame = get_robot_state().base_frame


def locate(queries):
    for qq in queries:
        f = localize_object(qq, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None


def q(vals, frac):
    o = sorted(float(v) for v in vals)
    return o[int((len(o) - 1) * frac)]


stove = locate(("flat stove", "black stove", "stove burner"))
knob = locate(("black knob", "stove knob", "black cylindrical knob"))

kx = ky = kz = None
if stove is not None:
    result["localized"] = True
    sp = stove.point_cloud.points
    sxmin = q([p[0] for p in sp], 0.02)
    symin = q([p[1] for p in sp], 0.02)
    symax = q([p[1] for p in sp], 0.98)
    stop = q([p[2] for p in sp], 0.9)
    result["stove_bounds"] = [sxmin, symin, symax, stop]
    kx = sxmin - 0.10
    ky = symin + 0.4 * (symax - symin)
    kz = stop + 0.04
if knob is not None and len(knob.point_cloud.points) < 4000:
    kp = knob.point_cloud.points
    cx = q([p[0] for p in kp], 0.5)
    cy = q([p[1] for p in kp], 0.5)
    cz = q([p[2] for p in kp], 0.9)
    if stove is None or (abs(cx - kx) < 0.25 and abs(cy - ky) < 0.25):
        kx, ky, kz = cx, cy, cz
        result["knob_found"] = True

if kx is None:
    result["notes"] = "nothing found"
else:
    result["knob"] = [kx, ky, kz]
    quat = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
    close_gripper()
    oks = []

    def go(x, y, z, tol=0.02, steps=200):
        try:
            r = move_to_pose(Pose((x, y, z), quat, base_frame), tolerance=tol, max_steps=steps)
            oks.append(bool(r.ok))
            return bool(r.ok)
        except Exception as e:
            oks.append(False)
            result["notes"] = "err: " + str(e)
            return False

    z_hi = kz + 0.12
    z_low = kz - 0.01
    y_start = ky - 0.08
    y_end = ky + 0.08
    go(kx, y_start, z_hi)
    if go(kx, y_start, z_low):
        n = 8
        for i in range(1, n + 1):
            yy = y_start + (y_end - y_start) * i / n
            if not go(kx, yy, z_low, 0.015, 120):
                break
    go(kx, y_end, z_hi)
    result["moved"] = any(oks)
    result["move_oks"] = oks
