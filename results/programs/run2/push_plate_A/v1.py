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
        ee = state.arms["primary"].ee_pose if hasattr(state, "arms") else None
        quat = list(ee.quaternion_wxyz) if ee is not None else [0, 1, 0, 0]
        close_gripper()
        # Push direction: from plate toward stove (in robot base frame, horizontal)
        dx = sp[0] - pp[0]
        dy = sp[1] - pp[1]
        n = math.sqrt(dx * dx + dy * dy)
        if n < 1e-6:
            n = 1.0
        ux, uy = dx / n, dy / n
        z_hover = pp[2] + 0.12
        z_push = pp[2] + 0.02
        start = [pp[0] - ux * 0.12, pp[1] - uy * 0.12]
        r1 = move_to_pose(pose_at([start[0], start[1], z_hover], quat), max_steps=200)
        r2 = move_to_pose(pose_at([start[0], start[1], z_push], quat), max_steps=200)
        result["moved_above"] = bool(r1.ok and r2.ok)
        # push most of the way toward the stove front (stop short of the stove center)
        dist = max(0.0, n - 0.12)
        goal = [pp[0] + ux * dist * 0.8, pp[1] + uy * dist * 0.8]
        r3 = move_to_pose(pose_at([goal[0], goal[1], z_push], quat), max_steps=300)
        result["pushed"] = bool(r3.ok)
        move_to_pose(pose_at([goal[0], goal[1], z_hover], quat), max_steps=200)
        result["done"] = True
except Exception as e:
    result["error"] = str(e)
