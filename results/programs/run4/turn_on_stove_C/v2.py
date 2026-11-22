import math

def near_ok(r):
    return r.ok or (r.error is not None and r.error.details.get("joint_error_rad", 9) < 0.05)

state = get_robot_state()
base_frame = state.base_frame
downward = state.end_effector_poses["primary"].quaternion_wxyz
result = {"localized": False, "grasped": False, "rotated": False}

def locate(queries):
    for q in queries:
        f = localize_object(q, camera_name="agentview", target_frame=base_frame)
        if f.ok and len(f.point_cloud.points) >= 5:
            c = f.geometry.pose.position
            ext = f.geometry.extents
            small = all(float(e) < 0.15 for e in ext)
            if c[2] < 0.15 and small:
                return f
    return None

found = locate(("black stove knob", "black knob", "stove knob", "knob", "round black handle"))
if found is None:
    raise Exception("knob not found")
pts = found.point_cloud.points
zs = sorted(float(p[2]) for p in pts)
c = found.geometry.pose.position
top_z = zs[int((len(zs) - 1) * 0.9)]
result["localized"] = True
result["center"] = [float(v) for v in c]
result["top_z"] = top_z

open_gripper()
hover_z = 0.30
r = move_to_pose(Pose((c[0], c[1], hover_z), downward, base_frame), tolerance=0.02, max_steps=300)
result["hover"] = near_ok(r)
if not result["hover"]:
    raise Exception("hover failed")
r = move_to_pose(Pose((c[0], c[1], top_z + 0.08), downward, base_frame), tolerance=0.02, max_steps=250)
r = move_to_pose(Pose((c[0], c[1], max(top_z - 0.02, 0.0)), downward, base_frame), tolerance=0.015, max_steps=250)
result["descended"] = near_ok(r)
close_gripper()
result["grasped"] = True

# rotate wrist in the opposite direction (previous positive twist did not turn the stove on)
for i, d in enumerate((-0.9, -0.7)):
    j = [float(v) for v in get_robot_state().joint_positions["primary"]]
    j[-1] += d
    r = move_to_joints(j, tolerance=0.02, max_steps=250)
    result["rot%d" % i] = near_ok(r)
result["rotated"] = result.get("rot0", False)

open_gripper()
cur = get_robot_state().end_effector_poses["primary"].position
move_to_pose(Pose((cur[0], cur[1], hover_z), downward, base_frame), tolerance=0.03, max_steps=250)
