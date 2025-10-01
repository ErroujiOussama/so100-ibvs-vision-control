import gz.transport

node = gz.transport.Node()
pub = node.advertise("/world/empty/set_pose", "gz.msgs.Pose")

msg = gz.msgs.Pose()
msg.name = "so101::gripper"
msg.position.x = 1.0
msg.position.y = 0.0
msg.position.z = 0.5
msg.orientation.w = 1.0

pub.publish(msg)
