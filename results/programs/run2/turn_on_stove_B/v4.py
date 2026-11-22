def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]

result = {"localized": False, "knob": False, "moved_above": False, "swept": False}
base_frame = get_robot_state().base_frame
down = get_robot_state().end_effector_poses["primary"].quaternion_wxyz

def go(x, y, z, tol=0.03, steps=200):
    r = move_to_pose(Pose((x, y, z), down, base_frame), tolerance=tol, max_steps=steps)
    return r.ok

def locate(queries):
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None

stove = locate(("flat stove", "black stove", "stove"))
knob = locate(("stove knob", "black knob", "small knob"))
close_gripper()
count = 0

if stove is not None:
    result["localized"] = True
    spts = stove.point_cloud.points
    surf_z = quantile((p[2] for p in spts), 0.5)
    sx0 = quantile((p[0] for p in spts), 0.03)
    sx1 = quantile((p[0] for p in spts), 0.97)
    sy0 = quantile((p[1] for p in spts), 0.03)
    sy1 = quantile((p[1] for p in spts), 0.97)

    targets = []
    if knob is not None:
        kp = knob.point_cloud.points
        kx = quantile((p[0] for p in kp), 0.5)
        ky = quantile((p[1] for p in kp), 0.5)
        kz = quantile((p[2] for p in kp), 0.9)
        # accept knob only if it is near the stove footprint
        if sx0 - 0.1 <= kx <= sx1 + 0.1 and sy0 - 0.1 <= ky <= sy1 + 0.1:
            result["knob"] = True
            targets.append((kx, ky, kz))
    if not targets:
        # fall back: stove front-edge candidates (knob usually at the edge)
        yc = 0.5 * (sy0 + sy1)
        xc = 0.5 * (sx0 + sx1)
        for (tx, ty) in ((sx0 + 0.02, yc), (xc, sy0 + 0.02), (xc, sy1 - 0.02), (sx1 - 0.02, yc)):
            targets.append((tx, ty, surf_z + 0.02))

    for (kx, ky, kz) in targets:
        hover = kz + 0.15
        if not go(kx, ky - 0.05, hover):
            continue
        result["moved_above"] = True
        low = None
        for dz in (0.02, 0.035, 0.05, 0.07):
            if go(kx, ky - 0.05, kz + dz, steps=200):
                low = kz + dz
                break
        if low is None:
            continue
        # sweep through the knob in y, then in x, to rotate it
        for (ax, ay) in ((kx, ky), (kx, ky + 0.05), (kx, ky - 0.05),
                         (kx - 0.05, ky), (kx + 0.05, ky), (kx, ky)):
            if go(ax, ay, low, steps=150):
                count += 1
        go(kx, ky, hover)
        if result["knob"]:
            break
else:
    result["error"] = "stove not found"

result["swept"] = count > 0
result["sweep_steps"] = count
