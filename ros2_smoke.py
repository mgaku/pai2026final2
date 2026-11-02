import os,time,json,sys
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
assert os.environ.get('ROS_DISTRO')=='humble',repr(os.environ.get('ROS_DISTRO'))
rclpy.init();a=Node('smoke_talker');b=Node('smoke_listener');received=[]
pub=a.create_publisher(String,'/pai/smoke',10)
sub=b.create_subscription(String,'/pai/smoke',lambda m:received.append(m.data),10)
e=SingleThreadedExecutor();e.add_node(a);e.add_node(b)
end=time.monotonic()+20
while not received and time.monotonic()<end:
    m=String();m.data='ROS2_HUMBLE_OK';pub.publish(m);e.spin_once(timeout_sec=.1)
assert 'ROS2_HUMBLE_OK' in received,'ROS2 pub/sub failed'
print(json.dumps({'test':'ROS2_HUMBLE_OK','ROS_DISTRO':os.environ['ROS_DISTRO'],'python':sys.version,'received':received[0]}))
e.shutdown();a.destroy_node();b.destroy_node();rclpy.shutdown()