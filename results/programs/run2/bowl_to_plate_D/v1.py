def quantile(values, fraction):
    ordered = sorted(float(v) for v in values)
    return ordered[int((len(ordered) - 1) * fraction)]

def locate(queries, camera_name="agentview"):
    base_frame = get_robot_state().base_frame
    for q in queries:
        f = localize_object(q, camera_name=camera_name, target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 20:
            return f
    return None

result = {"localized": False, "grasped": False, "placed": False}
base_frame = get_robot_state().base_frame

try:
    bowl = locate(("black bowl", "akita black bowl", "bowl"))
    plate = locate(("white plate", "plate", "round plate"))
    if bowl is None or plate is None:
        raise ValueError("object not found")
    result["localized"] = True

    bpts = bowl.point_cloud.points
    bc = (quantile([p[0] for p in bpts], 0.5), quantile([p[1] for p in bpts], 0.5))
    btop = quantile([p[2] for p in bpts], 0.95)
    ppts = plate.point_cloud.points
    pc = (quantile([p[0] for p in ppts], 0.5), quantile([p[1] for p in ppts], 0.5))
    psurf = quantile([p[2] for p in ppts], 0.9)

    quat = get_robot_state().end_effector_poses["primary"].quaternion_wxyz
    open_gripper()

    # grasp the bowl rim: offset toward the near edge is unknown, so grasp at the top of the bowl
    gz = btop - 0.02
    move_to_pose(Pose((bc[0], bc[1], gz + 0.12), quat, base_frame), tolerance=0.02, max_steps=250)
    move_to_pose(Pose((bc[0], bc[1], gz), quat, base_frame), tolerance=0.015, max_steps=250)
    close_gripper()

    lift_z = gz + 0.18
    lift = move_to_pose(Pose((bc[0], bc[1], lift_z), quat, base_frame), tolerance=0.02, max_steps=250)
    result["grasped"] = bool(lift.ok)

    move_to_pose(Pose((pc[0], pc[1], lift_z), quat, base_frame), tolerance=0.02, max_steps=300)
    move_to_pose(Pose((pc[0], pc[1], psurf + 0.08), quat, base_frame), tolerance=0.02, max_steps=250)
    open_gripper()
    result["placed"] = True
    move_to_pose(Pose((pc[0], pc[1], lift_z), quat, base_frame), tolerance=0.03, max_steps=250)
except Exception as e:
    result["error"] = str(e)
