#!/usr/bin/python
### plot distribution of loop candidates
import matplotlib.pyplot as plt
import matplotlib
from pose_graph_msgs.msg import LoopCandidateArray, LoopCandidate
from pose_graph_msgs.msg import PoseGraph, PoseGraphEdge
import rosbag
import argparse
import math
import numpy as np
import os
import key_handling


def extract_loop_candidates(bag_name, topic_name):
  pair_lc_map = {}
  with rosbag.Bag(bag_name) as bag:
    for topic, message, t in bag.read_messages(topics=[topic_name]):
      for candidate in message.candidates:
        pfx_from, _ = key_handling.split_pg_key(candidate.key_from)
        pfx_to, _ = key_handling.split_pg_key(candidate.key_to)
        if (pfx_from, pfx_to) in pair_lc_map:
          pair_lc_map[(pfx_from, pfx_to)].append(candidate)
        elif (pfx_to, pfx_from) in pair_lc_map:
          pair_lc_map[(pfx_to, pfx_from)].append(candidate)
        else:
          pair_lc_map[(pfx_from, pfx_to)] = [candidate]
  return pair_lc_map


def extract_loop_closures(bag_name, topic_name):
  pair_lc_map = {}
  with rosbag.Bag(bag_name) as bag:
    for topic, message, t in bag.read_messages(topics=[topic_name]):
      for lc in message.edges:
        pfx_from, _ = key_handling.split_pg_key(lc.key_from)
        pfx_to, _ = key_handling.split_pg_key(lc.key_to)
        if (pfx_from, pfx_to) in pair_lc_map:
          pair_lc_map[(pfx_from, pfx_to)].append(lc)
        elif (pfx_to, pfx_from) in pair_lc_map:
          pair_lc_map[(pfx_to, pfx_from)].append(lc)
        else:
          pair_lc_map[(pfx_from, pfx_to)] = [lc]
  return pair_lc_map


def plot_lc_distribution(loops, ax):
  # Pie chart, where the slices will be ordered and plotted counter-clockwise:
  labels = []
  sizes = []

  for l in loops:
    label_str = l[0] + l[1] + " : " + str(len(loops[l]))
    labels.append(label_str)
    sizes.append(len(loops[l]))
  ax.pie(sizes, labels=labels, autopct='%1.1f%%', startangle=90)
  ax.axis('equal')  # Equal aspect ratio ensures that pie is drawn as a circle.


def merge_loops(d1, d2):
  for k in d2:
    if (k[0], k[1]) in d1:
      d1[(k[0], k[1])].extend(d2[k])
    elif (k[1], k[0]) in d1:
      d1[(k[1], k[0])].extend(d2[k])
    else:
      d1[k] = d2[k]
  return d1


def main():
    # Parse user input
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--folder", default="/data/ros/base1_2021-09-14-09-58-00_lamp_adapt_t3/rosbag/", help="folder containing the bag files")

    options = parser.parse_args()
    loop_gen_topic = "/base1/lamp/loop_generation/loop_candidates"
    loop_prior_topic = "/base1/lamp/prioritization/prioritized_loop_candidates"
    loop_queue_topic = "/base1/lamp/loop_candidate_queue/prioritized_loop_candidates"
    loop_closure_topic = "/base1/lamp/laser_loop_closures"

    loop_gen = {}
    loop_prior = {}
    loop_queue = {}
    loop_closures = {}
    for file in os.listdir(options.folder):
      if file.startswith("base1_lamp"):
        filestr = options.folder + file
        print("Reading from %s" % (filestr))
        lg = extract_loop_candidates(filestr, loop_gen_topic)
        lp = extract_loop_candidates(filestr, loop_prior_topic)
        lq = extract_loop_candidates(filestr, loop_queue_topic)
        lc = extract_loop_closures(filestr, loop_closure_topic)

        loop_gen = merge_loops(loop_gen, lg)
        loop_prior = merge_loops(loop_prior, lp)
        loop_queue = merge_loops(loop_queue, lq)
        loop_closures = merge_loops(loop_closures, lc)

    fig, axs = plt.subplots(2, 2)
    plot_lc_distribution(loop_gen, axs[0, 0])
    axs[0, 0].set_title("Loop Generated")
    plot_lc_distribution(loop_prior, axs[0, 1])
    axs[0, 1].set_title("Loop Prioritized")
    plot_lc_distribution(loop_queue, axs[1, 0])
    axs[1, 0].set_title("Loop in Queue")
    plot_lc_distribution(loop_closures, axs[1, 1])
    axs[1, 1].set_title("Loop Closures")

    plt.show()


if __name__ == '__main__':
    main()
