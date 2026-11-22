result = {"localized": False, "moved_above": False, "pushed": False, "done": False}

def pose_at(p, quat):
    return Pose(position=list(p), quaternion_wxyz=list(quat), frame="robot_base")

try:
    plate = localize_object("plate")
    stove = localize_object("flat stove")
    result["localized"] = bool(plate.ok and stove.ok)
    if plate.ok and stove.ok:
        pp = list(plate.geometry.pose.position)
        sp = list(stove.geometry.pose.position)
        state = get_robot_state()
        ee = state.arms["primary"].ee_pose
        quat = list(ee.quaternion_wxyz)
        close_gripper()
        # Stove front region lies about 0.36 m in +x from the stove centre
        tx = sp[0] + 0.36
        ty = sp[1]
        dx = tx - pp[0]
        dy = ty - pp[1]
        n = math.sqrt(dx * dx + dy * dy)
        if n < 1e-6:
            n = 1.0
        ux, uy = dx / n, dy / n
        z_hover = pp[2] + 0.15
        z_push = pp[2] + 0.03
        start = [pp[0] - ux * 0.15, pp[1] - uy * 0.15]
        r1 = move_to_pose(pose_at([start[0], start[1], z_hover], quat), max_steps=200)
        r2 = move_to_pose(pose_at([start[0], start[1], z_push], quat), max_steps=200)
        result["moved_above"] = bool(r1.ok and r2.ok)
        ok = True
        for f in [0.35, 0.7, 1.0]:
            gx = start[0] + ux * (n * f + 0.15)
            gy = start[1] + uy * (n * f + 0.15)
            r = move_to_pose(pose_at([gx, gy, z_push], quat), max_steps=200)
            ok = ok and bool(r.ok)
        result["pushed"] = ok
        move_to_pose(pose_at([tx, ty, z_hover], quat), max_steps=200)
        result["done"] = True
except Exception as e:
    result["error"] = str(e)
