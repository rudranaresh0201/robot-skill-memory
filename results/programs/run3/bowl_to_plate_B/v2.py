def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]

def locate(queries, camera_name="agentview"):
    bf = get_robot_state().base_frame
    for q in queries:
        f = localize_object(q, camera_name=camera_name, target_frame=bf)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None

def go(pose, tol=0.03, steps=300):
    r = move_to_pose(pose, tolerance=tol, max_steps=steps)
    if r.ok:
        return True
    # a near-converged timeout: retry once from the current (close) position
    r = move_to_pose(pose, tolerance=tol, max_steps=steps)
    return r.ok

result = {"localized": False, "grasped": False, "placed": False}
base_frame = get_robot_state().base_frame
downward = get_robot_state().end_effector_poses["primary"].quaternion_wxyz

bowl = locate(("black bowl", "akita black bowl", "bowl"))
plate = locate(("plate", "white plate", "round plate"))
if bowl is None or plate is None:
    result["error"] = "localization failed"
else:
    result["localized"] = True
    bpts = bowl.point_cloud.points
    bcx = quantile([p[0] for p in bpts], 0.5)
    bcy = quantile([p[1] for p in bpts], 0.5)
    btop = quantile([p[2] for p in bpts], 0.95)
    ppts = plate.point_cloud.points
    pc = (quantile([p[0] for p in ppts], 0.5), quantile([p[1] for p in ppts], 0.5))
    ptop = quantile([p[2] for p in ppts], 0.9)

    # grasp the rim on the min-y side so the fingers straddle the wall
    rim_y = quantile([p[1] for p in bpts], 0.05) + 0.008
    grasp_z = btop - 0.02
    open_gripper()
    ok = go(Pose((bcx, rim_y, grasp_z + 0.12), downward, base_frame), 0.03, 300)
    if ok:
        ok = go(Pose((bcx, rim_y, grasp_z), downward, base_frame), 0.03, 300)
    result["approach"] = ok
    if ok:
        close_gripper()
        lift_z = grasp_z + 0.18
        r = go(Pose((bcx, rim_y, lift_z), downward, base_frame), 0.03, 300)
        result["lifted"] = r
        if r:
            result["grasped"] = True
            off_y = rim_y - bcy
            tx, ty = pc[0], pc[1] + off_y
            r2 = go(Pose((tx, ty, lift_z), downward, base_frame), 0.03, 400)
            result["transported"] = r2
            if r2:
                r3 = go(Pose((tx, ty, ptop + 0.09), downward, base_frame), 0.03, 300)
                if r3:
                    open_gripper()
                    state = get_robot_state()
                    for t in range(20):
                        step(RobotAction(arms={"primary": ArmCommand(
                            "joint_position",
                            state.joint_positions["primary"],
                            gripper_position=1.0,
                            embodiment=state.embodiment,
                        )}))
                    result["placed"] = True
