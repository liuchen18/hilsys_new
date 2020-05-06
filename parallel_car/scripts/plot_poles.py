#!/usr/bin/env python

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":

    # file to read for pole length data
    file_name = "pole_length.csv"

    # number of poles
    pole_num = 6

    # data in DataFrame
    csv_data = pd.read_csv(file_name)

    # ndarray for pole length list
    pole_length_array = csv_data.values

    # rows in plot
    plot_row = 3
    plot_col = pole_num/plot_row

    # get figure handle
    fig, axes = plt.subplots(plot_row, plot_col, figsize=(50, 20))

    # color list
    color_list = ['black', 'green', 'red', 'blue', 'yellow', 'pink']

    # bounds
    lower_bound = 0.35
    upper_bound = 0.55

    # get time sequence
    time_array = pole_length_array[:,0]
    # subtract start offest
    time_start = time_array[0]
    time_array = time_array - time_start
    
    for row_idx in range(plot_row):
        for col_idx in range(plot_col):
            num_idx = row_idx + 3 * col_idx
            # get the handle of the subplot
            ax = axes[row_idx][col_idx]
            # plot the data
            ax.plot(time_array, pole_length_array[:,num_idx+1], label='pole'+str(num_idx+1), color=color_list[num_idx])
            # set y grids
            ax.set_yticks([lower_bound, upper_bound], minor=True)
            ax.set_yticks(np.linspace(0.3, 0.6, num=4), minor=False)
            ax.yaxis.grid(True, which='minor')
            ax.yaxis.grid(True, which='major')

            # show all the things
            ax.legend()

    # show the plot
    plt.savefig('./poles.jpg')
    plt.show()