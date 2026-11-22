import math

base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "stove_localized": False, "pushed": False}


def find(queries):
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None


plate = find(("plate", "white plate", "round plate"))
stove = find(("black stove", "flat stove", "stove burner"))

if plate is None:
    result["error"] = "plate not found"
else:
    result["localized"] = True
    c = plate.geometry.pose.position
    ext = plate.geometry.extents
    radius = max(0.07, min(0.12, max(ext[0], ext[1]) / 2.0))
    low_z = min(float(p[2]) for p in plate.point_cloud.points)
    push_z = low_z + 0.02

    if stove is not None:
        result["stove_localized"] = True
        s = stove.geometry.pose.position
        # stove "front" lies on the +x side of the stove (toward the camera),
        # about 0.33 m from its center in the table layout
        tx = s[0] + 0.33
        ty = s[1]
    else:
        tx = c[0] - 0.10
        ty = c[1] + 0.23
    dx = tx - c[0]
    dy = ty - c[1]
    dist = math.sqrt(dx * dx + dy * dy)
    result["push_dist"] = dist
    if dist < 0.02:
        result["pushed"] = True
    else:
        ux, uy = dx / dist, dy / dist
        sx = c[0] - ux * (radius + 0.04)
        sy = c[1] - uy * (radius + 0.04)
        ex = sx + ux * (dist + 0.03)
        ey = sy + uy * (dist + 0.03)

        close_gripper()
        r1 = move_to_pose(Pose((sx, sy, push_z + 0.12), downward, base_frame),
                          tolerance=0.03, max_steps=300)
        result["above_start"] = r1.ok
        if r1.ok:
            r2 = move_to_pose(Pose((sx, sy, push_z), downward, base_frame),
                              tolerance=0.02, max_steps=300)
            result["descended"] = r2.ok
            if r2.ok:
                ok_all = True
                for f in (0.35, 0.7, 1.0):
                    mx = sx + (ex - sx) * f
                    my = sy + (ey - sy) * f
                    r = move_to_pose(Pose((mx, my, push_z), downward, base_frame),
                                     tolerance=0.03, max_steps=250)
                    if not r.ok:
                        ok_all = False
                        result["push_fail_at"] = f
                        break
                result["pushed"] = ok_all
                move_to_pose(Pose((ex, ey, push_z + 0.12), downward, base_frame),
                             tolerance=0.04, max_steps=200)
