

import matplotlib.pyplot as plt
import numpy as np

# highD Data for general and interstate
# partial_costs = ["SD", "UB", "MSL", "TF", "S", "DFL", "EL"]
# time_of_evaluation = [1810, 1875, 1308, 1285, 2359, 19372, 2874]  # Time of evaluation in seconds
# evaluated_vehicles =  1000 #max(time_of_evaluation)  # Use the maximum time as reference for scaling

# # sinD Data for intersection
partial_costs = ["SS", "RL"]
time_of_evaluation = [679, 937]
evaluated_vehicles = 335  # Maximum number of evaluated vehicles (horizontal dashed line)


# Bar positions and height
x_positions = np.arange(len(partial_costs))
bar_heights = time_of_evaluation  # Directly plot the time of evaluation

# Plot
plt.figure(figsize=(10, 6))
bars = plt.bar(x_positions, bar_heights, color="lightgray", edgecolor="gray", width=0.8)

# Dashed horizontal line for maximum time
#plt.axhline(y=evaluated_vehicles, color="black", linestyle="--", linewidth=1.2, label="Maximum Evaluation Time")

# Add annotations on bars
for bar, time in zip(bars, time_of_evaluation):
    height = bar.get_height()
    plt.text(bar.get_x() + bar.get_width() / 2, height/2, f"{time}", ha="center", va="bottom", fontsize=14, color="black")

# X-axis labels
plt.xticks(x_positions, partial_costs, rotation=45, ha="right", fontsize=12)

# Labels and title
plt.ylabel("Time (s)", fontsize=14)
#plt.title("Time of Evaluation for Partial Costs", fontsize=16, weight="bold")
plt.legend(loc="upper right", fontsize=12)

# Grid and layout
plt.grid(axis="y", linestyle="--", alpha=0.5)
plt.tight_layout()

# Show the plot
plt.show()
