"""
Description:  
    - Boxplot from npz 
Author: 
    - Matteo Palieri, NASA Jet Propulsion Laboratory
"""

import os
import sys 
import numpy as np 
import seaborn as sns
from matplotlib import pyplot as plt

def main():

    if len(sys.argv)<2:
        print("Minimal Usage: python boxplot_from_npz.py method1")
        print("Example Usage: python boxplot_from_npz.py method1 method2 method3 method4")
        sys.exit(1)  
    import pdb; pdb.set_trace()
    methods, data = [], []
    for i in range(len(sys.argv)):
        if i!=0:
            methods.append(sys.argv[i])

    distances = [100, 200, 300] 
    for distance in distances: 
        for method in methods: 
            filename = os.getcwd() + "/" + str(distance) + "/error_array_" + method + ".npz"
            data.append(np.load(filename))
        data.append([]) 
        
    bplot = sns.boxplot(data=[d for d in data], width=0.5)   
    colors = ["red", "green", "blue", "yellow"]
    
    color_counter = 0
    for i in range(len(data)-len(distances)): 
        mybox = bplot.artists[i]
        mybox.set_facecolor(colors[color_counter])
        color_counter = color_counter + 1
        if color_counter == len(methods): 
            color_counter=0   
    plt.show()

if __name__=="__main__": 
    main()