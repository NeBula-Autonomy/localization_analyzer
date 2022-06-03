"""
Description:  
    - Converts a pose_graph.bag to an odometry.bag   
Author: 
    - Matteo Palieri, NASA Jet Propulsion Laboratory
"""

import sys
import rospy 
import rosbag 
from nav_msgs.msg import Odometry

def extract_last_msg_from_bag(bag, topic):
    last_msg = None
    for topic, msg, t in bag.read_messages(topics=[topic]):
        last_msg = msg
    return last_msg

def main():

    if len(sys.argv)<5:
        print("Example Usage: python pose_graph_to_odometry.py pose_graph.bag pose_graph_topic odometry.bag odometry_topic")
        sys.exit(1)

    pose_graph_bag = rosbag.Bag(sys.argv[1])
    pose_graph_topic = sys.argv[2]
    out_bag = sys.argv[3]
    out_topic = sys.argv[4]

    pose_graph = extract_last_msg_from_bag(pose_graph_bag, pose_graph_topic)

    if pose_graph is not None: 
        with rosbag.Bag(out_bag, "w") as odometry_bag: 
            for node in pose_graph.nodes:
                if node.ID=="odom_node":
                    msg = Odometry()
                    msg.header.stamp = node.header.stamp
                    msg.pose.pose = node.pose 
                    odometry_bag.write(out_topic, msg, msg.header.stamp)
        odometry_bag.close()
    else: 
        print("Unable to retrieve pose_graph from pose_graph.bag on specified pose_graph_topic")

if __name__=="__main__":
    main()