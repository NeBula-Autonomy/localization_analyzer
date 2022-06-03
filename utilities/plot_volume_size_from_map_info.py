import sys 
import rosbag   
import matplotlib
import numpy as np
import matplotlib.pyplot as plt

matplotlib.rcParams['mathtext.fontset'] = 'stix'
matplotlib.rcParams['font.family'] = 'Arial'
size, labelsize = 20, 20
params = {'legend.fontsize': 'large',
          'figure.figsize': (20,8),
          'axes.labelsize': labelsize,
          'axes.titlesize': labelsize,
          'xtick.labelsize': size*0.75,
          'ytick.labelsize': size*0.75,
          'axes.titlepad': 25}
plt.rcParams.update(params)

def plot_volume_size(volume, size, timestamps): 
    fig, ax1 = plt.subplots()
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('Volume (m^3)', color='tab:red')
    ax1.plot(timestamps, volume, color='tab:red', linewidth=4)
    ax1.tick_params(axis='y', labelcolor='tab:red')
    ax2 = ax1.twinx()  
    ax2.set_ylabel('Number of points in map', color='tab:blue')  
    ax2.plot(timestamps, size, color='tab:blue', linewidth=4)
    ax2.tick_params(axis='y', labelcolor='tab:blue')
    fig.tight_layout()  
    plt.show()

def main(): 
    if len(sys.argv)<2:
        print("Example Usage: python plot_volume_size_from_map_info.py base_name")
        sys.exit(1)
    bag = rosbag.Bag("map_info.bag")
    volume, size, timestamps = [], [], []
    for topic, msg, t in bag.read_messages(topics=["/"+sys.argv[1]+"/lamp/map_info"]):
        size.append(msg.size) # TODO: Does this need scaling? 
        volume.append(msg.volume)
        timestamps.append(t.to_sec())
    timestamps = [el-timestamps[0] for el in timestamps]
    plot_volume_size(volume, size, timestamps)
        
if __name__ == "__main__": 
    main()