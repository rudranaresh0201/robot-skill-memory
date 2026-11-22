import math

base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "pushed": False}

plate = localize_object("plate", camera_name="agentview", target_frame=base_frame)
if not plate.ok or len(plate.point_cloud.points) < 20:
    plate = localize_object("white plate", camera_name="agentview", target_frame=base_frame)
if not plate.ok or len(plate.point_cloud.points) < 20:
    result["error"] = "plate not found"
else:
    result["localized"] = True
    c = plate.geometry.pose.position
    ext = plate.geometry.extents
    radius = max(0.07, min(0.12, max(ext[0], ext[1]) / 2.0))
    zs = sorted(float(p[2]) for p in plate.point_cloud.points)
    low_z = zs[0]
    push_z = low_z + 0.02

    # displacement from plate region to stove-front region (table-frame axes)
    dx, dy = 0.15, 0.26
    dist = math.sqrt(dx * dx + dy * dy)
    ux, uy = dx / dist, dy / dist

    sx = c[0] - ux * (radius + 0.04)
    sy = c[1] - uy * (radius + 0.04)
    ex = c[0] + dx - ux * (radius - 0.01)
    ey = c[1] + dy - uy * (radius - 0.01)

    close_gripper()
    r1 = move_to_pose(Pose((sx, sy, push_z + 0.12), downward, base_frame), tolerance=0.02, max_steps=250)
    result["above_start"] = r1.ok
    if r1.ok:
        r2 = move_to_pose(Pose((sx, sy, push_z), downward, base_frame), tolerance=0.015, max_steps=250)
        result["descended"] = r2.ok
        if r2.ok:
            ok_all = True
            for f in (0.5, 1.0):
                mx = sx + (ex - sx) * f
                my = sy + (ey - sy) * f
                r = move_to_pose(Pose((mx, my, push_z), downward, base_frame), tolerance=0.02, max_steps=250)
                if not r.ok:
                    ok_all = False
                    result["push_fail_at"] = f
                    break
            result["pushed"] = ok_all
            move_to_pose(Pose((ex, ey, push_z + 0.12), downward, base_frame), tolerance=0.03, max_steps=200)
