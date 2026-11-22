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
    bc = cloud_c = (quantile([p[0] for p in bpts], 0.5), quantile([p[1] for p in bpts], 0.5), quantile([p[2] for p in bpts], 0.5))
    btop = quantile([p[2] for p in bpts], 0.95)
    ppts = plate.point_cloud.points
    pc = (quantile([p[0] for p in ppts], 0.5), quantile([p[1] for p in ppts], 0.5))
    ptop = quantile([p[2] for p in ppts], 0.9)

    # grasp the bowl rim: offset in x toward the edge of the bowl
    bxs = [p[0] for p in bpts]
    rim_x = quantile(bxs, 0.05)
    # use the bowl's near-edge on the min-x side
    grasp_z = btop - 0.02
    open_gripper()
    ok = move_to_pose(Pose((rim_x, bc[1], grasp_z + 0.12), downward, base_frame), tolerance=0.02, max_steps=250).ok
    if ok:
        ok = move_to_pose(Pose((rim_x, bc[1], grasp_z), downward, base_frame), tolerance=0.015, max_steps=250).ok
    result["approach"] = ok
    if ok:
        close_gripper()
        lift_z = grasp_z + 0.18
        r = move_to_pose(Pose((rim_x, bc[1], lift_z), downward, base_frame), tolerance=0.02, max_steps=250)
        result["lifted"] = r.ok
        if r.ok:
            result["grasped"] = True
            off = rim_x - bc[0]
            r2 = move_to_pose(Pose((pc[0] + off, pc[1], lift_z), downward, base_frame), tolerance=0.02, max_steps=300)
            if r2.ok:
                r3 = move_to_pose(Pose((pc[0] + off, pc[1], ptop + 0.09), downward, base_frame), tolerance=0.02, max_steps=250)
                if r3.ok:
                    open_gripper()
                    state = get_robot_state()
                    for t in range(20):
                        step(RobotAction(arms={"primary": ArmCommand(
                            "joint_position",
                            state.joint_positions["primary"],
                            gripper_position=1.0,
                        )}))
                    result["placed"] = True
