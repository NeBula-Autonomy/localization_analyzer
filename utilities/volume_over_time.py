import sys
import rospy 
import rosbag 
import numpy as np
import matplotlib.pyplot as plt

def extract_last_msg_from_bag(bag, topic):
    last_msg = None
    for topic, msg, t in bag.read_messages(topics=[topic]):
        last_msg = msg
    return last_msg

def main():

  bag = rosbag.Bag(sys.argv[1])

  time_vec = []
  volume_vec = []
  points_vec = []

  for topic, msg, t in bag.read_messages():
    time_vec.append(t.to_sec())
    volume_vec.append(msg.volume)
    points_vec.append(float(msg.size))
    
  # Plot the results

  fig, ax1 = plt.subplots()

  color = 'tab:red'
  ax1.set_xlabel('time (s)')
  ax1.set_ylabel('Volume (m^3)', color=color)
  ax1.plot(time_vec, volume_vec, color=color)
  ax1.tick_params(axis='y', labelcolor=color)

  ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

  color = 'tab:blue'
  ax2.set_ylabel('# of points in map', color=color)  # we already handled the x-label with ax1
  ax2.plot(time_vec, points_vec, color=color)
  ax2.tick_params(axis='y', labelcolor=color)

  fig.tight_layout()  # otherwise the right y-label is slightly clipped
  plt.show()

  fig, ax3 = plt.subplots()
  color = 'tab:red'
  ax3.set_xlabel('# points')
  ax3.set_ylabel('Volume (m^3)', color=color)
  ax3.plot(points_vec, volume_vec, color=color)
  ax3.tick_params(axis='y', labelcolor=color)

  plt.show()



if __name__=="__main__":
    main()