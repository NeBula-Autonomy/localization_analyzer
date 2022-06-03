import matplotlib.pyplot as plt
import numpy as np
from matplotlib import colors
from matplotlib.ticker import PercentFormatter
import csv
import argparse


class LoopResult:
  def __init__(self, fitness, t_error, r_error):
    self.fitness = fitness
    self.t_error = t_error
    self.r_error = r_error


def read_result_csv(file_name):
  with open(file_name) as f:
    reader = csv.reader(f)
    next(reader, None)
    results = []
    for row in reader:
      results.append(LoopResult(float(row[0]), float(row[1]), float(row[2])))
    return results


def plot_histograms(true_positives, false_positives, n_bins=10):
  n_bins = 20
  fig, axs = plt.subplots(1, 1)

  true_positive_fitness = [x.fitness for x in true_positives]
  false_positives_fitness = [x.fitness for x in false_positives]

  # We can set the number of bins with the `bins` kwarg
  axs.hist(true_positive_fitness, alpha=0.5, bins=n_bins, label="true positives")
  axs.hist(false_positives_fitness, alpha=0.5, bins=n_bins, label="false positives")
  axs.legend()
  axs.set_xlabel("fitness score")
  plt.show()


def main():
    # Parse user input
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--true_positives", default="", help="csv of true positives")
    parser.add_argument("-f", "--false_positives", default="", help="csv of false positives")
    parser.add_argument("-n", "--n_bins", type=int, default=20, help="number of bins in fitness histogram")

    options = parser.parse_args()

    true_results = read_result_csv(options.true_positives)
    false_results = read_result_csv(options.false_positives)

    plot_histograms(true_results, false_results, options.n_bins)


if __name__ == '__main__':
    main()
