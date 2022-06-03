"""
Description:  
    - Get dataset duration
Author: 
    - Matteo Palieri, NASA Jet Propulsion Laboratory
"""

import os 
import sys
import numpy as np

def main(): 
    
    if len(sys.argv)<2:
        print("Example Usage: python get_dataset_duration.py path_to_dataset")
        sys.exit(1) 

    durations = []
    os.system("rosbag info -y -k duration " + sys.argv[1] + "/*.bag > durations.txt")
    with open("durations.txt") as infile: 
        for line in infile: 
            if "-" not in line:
                durations.append(float(line))
                
    print("Dataset duration: " + str(np.max(durations)))

if __name__=="__main__": 
    main()