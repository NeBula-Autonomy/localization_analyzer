"""
Description:  
    - Recovers fiducial calibration from pose_graph.bag  
Author: 
    - Matteo Palieri, NASA Jet Propulsion Laboratory
"""

import sys
import rosbag

def main():

    if len(sys.argv)<3:
        print("Example Usage: python recover_fiducial_calibration_from_pose_graph.py pose_graph.bag pose_graph_topic")
        sys.exit(1)

    pose_graph_bag = rosbag.Bag(sys.argv[1])
    pose_graph_topic = sys.argv[2]

    position, orientation = None, None

    for topic, msg, t in pose_graph_bag.read_messages(topics=[pose_graph_topic]): 
        for node in msg.nodes: 
            if (node.ID=="odom_node"): 
                position = node.pose.position
                orientation = node.pose.orientation
                break
        break 
        
    if position is not None and orientation is not None: 
        print("*******Fiducial Calibration*******\n")
        print("orientation: \n" + str(orientation) + "\n")
        print("position: \n" + str(position) + "\n")
        print("**********************************\n")    
    else: 
        print("Unable to recover fiducial calibration from pose_graph.bag")

    # TODO: Save to .ros folder for $ROBOT_NAME

if __name__=="__main__":
    main()