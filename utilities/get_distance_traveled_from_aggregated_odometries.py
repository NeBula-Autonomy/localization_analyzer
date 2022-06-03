"""
Description:  
    - Get distance traveled from aggregated odometries
Author: 
    - Matteo Palieri, NASA Jet Propulsion Laboratory
"""

import sys
import rosbag 
import numpy as np 

def main():

    if len(sys.argv)<2:
        print("Minimal Usage: python get_distance_traveled.py method1")
        print("Example Usage: python get_distance_traveled.py method1 method2 method3 method4")
        sys.exit(1)  

    methods = []
    
    for i in range(len(sys.argv)):
        if i!=0:
            methods.append(sys.argv[i]) 

    all_distances = {}

    for method in methods:  
        initialized = False
        distance_traveled = 0
        previous_position = None    
        for topic, msg, t in rosbag.Bag("aggregated_odometries.bag").read_messages(topics=[method]):
            if not initialized: 
                previous_position = msg.pose.pose.position 
                initialized = True
                pass       
            current_position = msg.pose.pose.position  
            dx = np.square(current_position.x - previous_position.x)
            dy = np.square(current_position.y - previous_position.y) 
            dz = np.square(current_position.z - previous_position.z)
            distance_traveled = distance_traveled + np.sqrt(dx + dy + dz) 
            previous_position = current_position    
        print("\nDistance traveled by " + method + " : " + str(distance_traveled) + " m\n")
        all_distances[method] = distance_traveled

if __name__ == "__main__": 
    main()