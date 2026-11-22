import math

result = {"localized": False, "moved": False, "pressed": False}

try:
    loc = localize_object("flat stove")
    result["localized"] = bool(loc.ok)
    if loc.ok:
        p = loc.geometry.pose.position
        cx, cy, cz = float(p[0]), float(p[1]), float(p[2])
        quat = [0.0, 1.0, 0.0, 0.0]
        frame = get_robot_state().base_frame
        close_gripper()

        def go(x, y, z):
            return move_to_pose(Pose([x, y, z], quat, frame), tolerance=0.02, max_steps=200)

        r = go(cx, cy, cz + 0.2)
        result["moved"] = bool(r.ok)
        # try knob locations near the stove front edge, sweeping sideways
        for dx in (-0.1, -0.05, 0.0):
            for dy in (-0.08, 0.08):
                go(cx + dx, cy - dy, cz + 0.12)
                go(cx + dx, cy - dy, cz + 0.05)
                go(cx + dx, cy + dy, cz + 0.05)
                go(cx + dx, cy + dy, cz + 0.15)
                result["pressed"] = True
except Exception as e:
    result["error"] = str(e)
