import pandas as pd
import matplotlib.pyplot as plt
import argparse
from io import StringIO

# Parse arguments
parser = argparse.ArgumentParser()
parser.add_argument('-d', '--dir', type=str, default="./data/")
parser.add_argument('-s', '--save', type=bool, default=False)
args = parser.parse_args()

# Read the data and filter the data
df = pd.read_csv(args.dir + '/lEE_d.csv')

parser.add_argument('-i','--initial_t', type=float, default=0.0)
parser.add_argument('-f','--final_t', type=float, default=df['dt'].iloc[-1])
args = parser.parse_args()

# Filter the data
filtered_df = df[(df['dt'] > args.initial_t) & (df['dt'] < args.final_t)]

# Plot the data
fig, ax = plt.subplots(figsize=(10, 6))

# Define the columns to plot and their respective styles
columns = ["p_lEE_x","p_lEE_y","p_lEE_z","pdot_lEE_x","pdot_lEE_y","pdot_lEE_z"]
labels = [r'$p_{lEE_x}$', r'$p_{lEE_y}$', r'$p_{lEE_z}$', r'$\dot{p}_{lEE_x}$', r'$\dot{p}_{lEE_y}$', r'$\dot{p}_{lEE_z}$']
linestyles = ['-', '--', '-.', ':', '-', '--', '-.', ':', '-', '--', '-.', ':', '-', '--', '-.', ':', '-', '--', '-.', ':']      #[' ', ' ', ' ', ' ', ' ', ' ']
markersizes = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]

# Plotting the columns against 'dt'
lines = []
for i, col in enumerate(columns):
    line, = ax.plot(filtered_df['dt'].values, filtered_df[col].values, linestyle=linestyles[i], markersize=markersizes[i], label=labels[i])
    lines.append(line)

# Define the interactive legend behavior
def on_legend_click(event):
    # Get the index of the legend line that was clicked
    legline = event.artist
    index = leglines.index(legline)
    
    # Toggle the visibility of the corresponding data line
    line = lines[index]
    line.set_visible(not line.get_visible())
    
    # Update the legend line's alpha to indicate if the line is active
    if line.get_visible():
        legline.set_alpha(1.0)
    else:
        legline.set_alpha(0.5)
    fig.canvas.draw()

# Connect the event with the handler
leg = ax.legend()
leglines = leg.get_lines()
for legline in leglines:
    legline.set_picker(10)  # Enable picking on the legend line

fig.canvas.mpl_connect('pick_event', on_legend_click)

plt.xlabel('time (s)')
plt.ylabel('joint position (rad)')
# plt.title('Swing Leg Trajectory using QP Solver')
plt.grid(True, alpha=0.3)
# plt.grid(True, axis='y', alpha=0.3)
plt.tight_layout()
if(args.save):
    plt.savefig(args.dir + '/qpos.png', dpi=300)
plt.show()