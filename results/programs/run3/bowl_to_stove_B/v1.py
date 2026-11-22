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
stove = locate(("flat stove", "black stove", "stove", "cooktop"))
if bowl is None or stove is None:
    result["error"] = "localization failed"
else:
    result["localized"] = True
    pts = bowl.point_cloud.points
    bc = (quantile([p[0] for p in pts], 0.5), quantile([p[1] for p in pts], 0.5), quantile([p[2] for p in pts], 0.5))
    top_z = quantile([p[2] for p in pts], 0.95)
    spts = stove.point_cloud.points
    sc = (quantile([p[0] for p in spts], 0.5), quantile([p[1] for p in spts], 0.5))
    stove_z = quantile([p[2] for p in spts], 0.9)

    # Grasp bowl rim: find the rim edge nearest to robot (min x) of the bowl
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    rim_x = min(xs) + 0.01
    rim_y = quantile(ys, 0.5)
    grasp_z = top_z - 0.015

    open_gripper()
    ok = move_to_pose(Pose((rim_x, rim_y, grasp_z + 0.12), downward, base_frame), tolerance=0.02, max_steps=250).ok
    if ok:
        ok = move_to_pose(Pose((rim_x, rim_y, grasp_z), downward, base_frame), tolerance=0.015, max_steps=250).ok
    result["approached"] = ok
    if ok:
        close_gripper()
        lift_z = grasp_z + 0.20
        l = move_to_pose(Pose((rim_x, rim_y, lift_z), downward, base_frame), tolerance=0.02, max_steps=250)
        result["lifted"] = l.ok
        if l.ok:
            result["grasped"] = True
            # offset so bowl center lands over stove center
            off = rim_x - bc[0]
            tx, ty = sc[0] + off, sc[1]
            t = move_to_pose(Pose((tx, ty, lift_z), downward, base_frame), tolerance=0.02, max_steps=300)
            result["transported"] = t.ok
            if t.ok:
                rel_z = stove_z + (grasp_z - (bc[2] - (bc[2] - min(p[2] for p in pts)))) + 0.04
                d = move_to_pose(Pose((tx, ty, rel_z), downward, base_frame), tolerance=0.02, max_steps=250)
                result["descended"] = d.ok
                open_gripper()
                state = get_robot_state()
                for i in range(15):
                    step(RobotAction(arms={"primary": ArmCommand(
                        "joint_position",
                        state.joint_positions["primary"],
                        gripper_position=1.0)}))
                result["placed"] = d.ok
                move_to_pose(Pose((tx, ty, lift_z), downward, base_frame), tolerance=0.03, max_steps=200)
