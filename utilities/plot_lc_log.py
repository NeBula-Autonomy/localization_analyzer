#!/usr/bin/python
### extract and plot from loop computation log file
import matplotlib.pyplot as plt
import matplotlib
from nav_msgs.msg import Odometry
import argparse
import numpy as np


def readAndPlotFitnessScore(filename, maxscore):
  # Opening file
  file = open(filename, 'r')
  scores = []
  for line in file:
    splt = line.split("ICP: Converged or max iterations reached, but score:")
    if len(splt) > 1:
      splt1 = splt[1].split()
      splt2 = splt1[0].split(",")
      if float(splt2[0]) < maxscore:
        scores.append(float(splt2[0]))
  file.close()

  fig, axs = plt.subplots(1, 1)
  axs.hist(scores, alpha=0.8, bins=20)
  axs.legend()
  axs.set_xlabel("fitness score")
  axs.set_title("Unaccepted loop closures")
  plt.show()


def main():
    # Parse user input
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--filename", default="/home/costar/Downloads/base1_2021-09-10-09-59-00_test/log/_base1_loop_computation.log", help="log file")
    parser.add_argument("-m", "--maxscore", type=int, default=1000, help="max fitness score to plot")

    options = parser.parse_args()
    print("Reading from %s" % (options.filename))

    readAndPlotFitnessScore(options.filename, options.maxscore)


if __name__ == '__main__':
    main()
