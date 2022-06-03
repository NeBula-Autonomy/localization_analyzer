"""
Description:  
    - Analyse a set of pose-graph bags to evaluate loop closure performance 
Author: 
    - Benjamin Morrell, NASA Jet Propulsion Laboratory
"""

import sys
import rosbag
import numpy as np
import ctypes
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose


keyBits = ctypes.sizeof(ctypes.c_uint64) * 8
chrBits = ctypes.sizeof(ctypes.c_ubyte) * 8
indexBits = keyBits - chrBits
chrMask = np.uint64(np.int64(~np.ubyte(0)) << indexBits)
indexMask = ~chrMask

# TODO - consider rotaitonal differences

def split_pg_key(pg_key):
    c_ = chr(np.ubyte(np.int64(np.uint64(pg_key) & chrMask) >> indexBits))
    j_ = np.uint64(pg_key) & indexMask
    return c_, j_
# In [79]: split_pg_key(7061644215716937728)
# Out[79]: ('b', 0)

def ComputeTransform(keys, node_poses):

  # Check keys exist 
  if keys[0] not in node_poses:
    print "error - key 1, ", split_pg_key(keys[0]), "not in node_poses"
  if keys[1] not in node_poses:
    print "error - key 2, ", split_pg_key(keys[1]), "not in node_poses"

  node1 = node_poses[keys[0]]
  node2 = node_poses[keys[1]]

  # initialize
  transform = Pose()
  # Fill information
  transform.position.x = node2.position.x - node1.position.x
  transform.position.y = node2.position.y - node1.position.y
  transform.position.z = node2.position.z - node1.position.z
  # TODO - analyse rotational difference
  transform.orientation.x = 0.0
  transform.orientation.y = 0.0
  transform.orientation.z = 0.0
  transform.orientation.w = 1.0

  return transform


def ComputeDifferenceTranslation(transform1, transform2):
  # compute the difference in transforms
  difference = np.sqrt((transform1.position.x - transform2.position.x)**2 +
                       (transform1.position.y - transform2.position.y)**2 +
                       (transform1.position.z - transform2.position.z)**2)

  return difference

def printPGKeys(keys):
  print "PG keys are:", split_pg_key(keys[0]), " and ", split_pg_key(keys[1])
  # print(*split_pg_key(keys[1]), sep='')

def AnalyzeLoopClosures(bag, robot_name):
    topic_list = ["/" + robot_name + "/lamp/pose_graph_to_optimize",
                  "/" + robot_name + "/lamp_pgo/optimized_values"]

    pg_to_opt = {}
    opt_vals = {}

    # Loop through topics in the bag
    for topic, msg, t in bag.read_messages(topics=topic_list):
        if topic == topic_list[0]:
          # pg to opt
          pg_to_opt[msg.header.seq] = msg
        else:
          opt_vals[msg.header.seq] = msg


    # Loop through the pg_to_opt
    seq_list = pg_to_opt.keys()

    loop_closures = {}
    new_loops = []
    node_poses_pre = {}
    node_poses_post = {}
    needed_keys = {}

    for seq in seq_list:
      factors = pg_to_opt[seq].edges
      nodes = pg_to_opt[seq].nodes

      # Find loop closures
      for edge in factors:
        if edge.type != 3: # loop closure factor
          continue

        # Have a loop closure 
        id = (edge.key_from,edge.key_to)

        if id not in loop_closures:
          loop_closures[id] = [edge.pose]
          new_loops.append(id)
          needed_keys[edge.key_from] = True
          needed_keys[edge.key_to] = True
          printPGKeys(id) 

      # Get pose-nodes
      for node in nodes:
        if node.key not in needed_keys.keys():
          continue
        node_poses_pre[node.key] = node.pose

      # Get opt node poses
      for opt_node in opt_vals[seq].nodes:
        if opt_node.key not in needed_keys.keys():
          continue
        node_poses_post[opt_node.key] = opt_node.pose

      # Loop through new loop closures
      for keys in new_loops:
        # get the transform before
        loop_closures[keys].append(ComputeTransform(keys,node_poses_pre))

        # Get the transform after optimization
        loop_closures[keys].append(ComputeTransform(keys,node_poses_post))

      # reset
      new_loops = []
      needed_keys = {}
      node_poses_pre = {}
      node_poses_post = {}

    # end loop

    # Go through loops to do analysis
    start_to_lc_edge = {}
    start_to_opt = {}
    opt_to_lc_edge = {}

    for keys in loop_closures.keys():
      transforms = loop_closures[keys]

      start_to_lc_edge[keys] = ComputeDifferenceTranslation(transforms[1],transforms[0])
      start_to_opt[keys] = ComputeDifferenceTranslation(transforms[1],transforms[2])
      opt_to_lc_edge[keys] = ComputeDifferenceTranslation(transforms[2],transforms[0])

    # Write stats to csv

    print "Completed processing"

    # plot
    fig = plt.figure()
    ax = fig.gca()

    ax.plot(start_to_lc_edge.values(), label='Difference from start to loop computation')
    ax.plot(start_to_opt.values(), label='Difference from start to optimized')
    # ax.plot(opt_to_lc_edge.values(), label='Difference between optimized and computed')
    ax.set_xlabel('loop')
    ax.set_ylabel('difference (m)')
    ax.grid(b=True, which='both', color='0.65', linestyle='-')

    plt.legend()

    plt.show()



def main():
    
    if len(sys.argv)<3:
        print("Example Usage: python pose_graph_loop_closure_analysis.py input.bag robot_names")
        sys.exit(1)  
    
    bag = rosbag.Bag(sys.argv[1])
    robot_name = sys.argv[2]
    
    AnalyzeLoopClosures(bag, robot_name)
    
    # with rosbag.Bag(out_bag, "w") as outbag: 
    #     outbag.write(topic, first_msg)
    # outbag.close()

    
if __name__=="__main__":
    main()
    