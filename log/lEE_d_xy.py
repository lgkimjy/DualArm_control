#!usr/bin/env python3
__author__ = "JunYoung Kim"
__email__ = "lgkimjy@kist.re.kr"
__date__ = "03/20/2024"

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


def plot_2D_plane(dir_path, timestamp, figure_size, columns, linestyles, linewidths, labels, xlabel, ylabel, title, saveoption=0, showoption=1):
    
    # Read the data and filter the data
    df = pd.read_csv(dir_path)
    df = df[(df['dt'] >= timestamp[0]) & (df['dt'] <= timestamp[1])]

    # Plot the data
    fig, ax = plt.subplots(figsize=(figure_size[0], figure_size[1]))

    for i, column in enumerate(columns):
        ax.plot(df[column[0]].values, df[column[1]].values, label=labels[i], linestyle=linestyles[i] ,linewidth=linewidths[i])
       
    plt.xlabel(xlabel[0], fontsize=xlabel[1])
    plt.ylabel(ylabel[0], fontsize=ylabel[1])
    plt.title(title[0], fontsize=title[1])
    plt.xticks(fontsize = figure_size[-1])
    plt.yticks(fontsize = figure_size[-1])
    plt.legend(fontsize = figure_size[-1])
    plt.grid(True, alpha=0.3)
    # plt.ylim(0.1, 0.18)
    plt.axis('equal')
    plt.tight_layout()
    
    if saveoption == 1:
        plt.savefig('../results/' + dir_path[8:-4] + '.png')
    elif saveoption == 2:
        plt.savefig('../results/' + dir_path[8:-4] + '.pdf')
    elif saveoption == 3:
        plt.savefig('../results/' + dir_path[8:-4] + '.svg')

    if showoption == 1:
        plt.show()


if __name__ == "__main__":
    timestamp = [0, 1000]          # [ start, end ]
    figure_size = [10, 5, 12]    # [ width, height, tick font size ]
    columns = [['p_lEE_y', 'p_lEE_z']]
    labels = ['p_lEE']
    linestyles = ['--', '-']
    linewidths = [3, 2]
    xlabel = ['Y [m]', 12]
    ylabel = ['Z [m]', 12]
    title = ['', 15]
    saveoption = 0  # 0: no save, 1: png, 2: pdf, 3: svg
    plot_2D_plane('./data/lEE_d.csv', timestamp, figure_size, columns, linestyles, linewidths, labels, xlabel, ylabel, title, saveoption)