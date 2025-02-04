# import matplotlib.pyplot as plt
# import numpy as np

# # highD Data for general and interstate
# partial_costs = ["SD", "UB", "MSL", "TF", "S", "DFL", "EL", "CEV", "SS", "RL"]
# compliance_values = [91.4, 96.2, 75.6, 95.2, 99.9, 97.7, 98.2, "100", 68.4, 100.0]
# evaluated_vehicles = 1000  # Maximum number of evaluated vehicles (horizontal dashed line)
# #TODO:
# # PARTIAL COST CEV was evaluated with exiD datasets (N = 24)
# # PARTIAL COST SS, RL was evaluated with sinD datasets (N = 335) 
# # OTHERS wer evaluated with highD datasets (N = 1000)
# # i want a figure that not only shows the percentage but somehow show the dataset used

# # sinD Data for intersection
# # partial_costs = ["SS", "RL"]
# # compliance_values = [68.4, 100.0]
# # evaluated_vehicles = 335  # Maximum number of evaluated vehicles (horizontal dashed line)


# # Bar positions and height
# x_positions = np.arange(len(partial_costs))
# bar_heights = [value * evaluated_vehicles / 100 for value in compliance_values]

# # Plot
# plt.figure(figsize=(8, 6))
# bars = plt.bar(x_positions, bar_heights, color="lightgray", edgecolor="gray", width=0.8)

# # Dashed horizontal line
# plt.axhline(y=evaluated_vehicles, color="black", linestyle="--", linewidth=1.2, label="Number of Evaluated Vehicles (highD)")

# # Add percentage annotations on bars
# for bar, percentage in zip(bars, compliance_values):
#     height = bar.get_height()
#     plt.text(bar.get_x() + bar.get_width() / 2, height/2 , f"{percentage:.1f}%", ha="center", va="bottom", fontsize=14, color="black")

# # X-axis labels
# plt.xticks(x_positions, partial_costs, rotation=45, ha="right", fontsize=12)

# # Labels and title
# #plt.ylabel("Number of Vehicles", fontsize=14)
# #plt.title("Rule Compliance by the Evaluated Vehicles", fontsize=16, weight="bold")
# #plt.legend(loc="upper right", fontsize=12)

# # Grid and layout
# plt.grid(axis="y", linestyle="--", alpha=0.5)
# plt.tight_layout()

# # Show the plot
# plt.show()

# import matplotlib.pyplot as plt
# import numpy as np

# # Data
# partial_costs = ["SD", "UB", "MSL", "TF", "S", "DFL", "EL", "CEV", "SS", "RL"]
# compliance_values = [91.4, 96.2, 75.6, 95.2, 99.9, 97.7, 98.2, 100.0, 68.4, 100.0]
# datasets = ["highD"] * 7 + ["exiD", "sinD", "sinD"]
# evaluated_vehicles_highD = 1000
# evaluated_vehicles_exiD = 24
# evaluated_vehicles_sinD = 332

# # Define positions
# x_positions = np.arange(len(partial_costs))
# bar_width = 0.6

# # Define dataset colors
# colors = {
#     "highD": "lightgray",
#     "exiD": "lightblue",
#     "sinD": "lightgreen"
# }
# bar_colors = [colors[dataset] for dataset in datasets]

# # Create the figure and axes
# plt.figure(figsize=(12, 8))

# # Plot bars
# bars = plt.bar(x_positions, compliance_values, color=bar_colors, edgecolor="black", width=bar_width)

# # Add percentage annotations on the bars
# for bar, percentage in zip(bars, compliance_values):
#     plt.text(
#         bar.get_x() + bar.get_width() / 2,
#         bar.get_height() + 1,
#         f"{percentage:.1f}%",
#         ha="center",
#         va="bottom",
#         fontsize=12,
#         weight="bold",
#         color="black"
#     )

# # Dashed horizontal lines for number of evaluated vehicles
# #plt.axhline(y=100, color="black", linestyle="--", linewidth=1.2, #label="Baseline (100%)"
#   #          )
# # plt.axhline(y=evaluated_vehicles_highD / 10, color="gray", linestyle="--", linewidth=1.2, label="highD")
# # plt.axhline(y=evaluated_vehicles_exiD / 10, color="blue", linestyle="--", linewidth=1.2, label="exiD")
# # plt.axhline(y=evaluated_vehicles_sinD / 10, color="green", linestyle="--", linewidth=1.2, label="sinD")

# # Customization
# plt.xticks(x_positions, partial_costs, rotation=45, fontsize=12)
# plt.yticks(fontsize=12)
# plt.xlabel("Partial Cost Functions", fontsize=14, weight="bold")
# plt.ylabel("Compliance Percentage (%)", fontsize=14, weight="bold")
# plt.title("Compliance Across Partial Cost Functions", fontsize=16, weight="bold", pad=20)
# plt.legend(loc="lower right", fontsize=12)

# # Add grid for better readability
# plt.grid(axis="y", linestyle="--", alpha=0.7)

# # Adjust layout
# plt.tight_layout()

# # Show the plot
# plt.show()


import matplotlib.pyplot as plt
import numpy as np

# Data
partial_costs = ["SD", "UB", "MSL", "TF", "ST", "DFL", "EL", "CEV", "SS", "RL", "G_I"]
compliance_values = [91.4, 96.2, 75.6, 95.2, 99.9, 97.7, 98.2, 100.0, 68.4, 100.0, 62.5]
datasets = ["highD"] * 7 + ["exiD", "sinD", "sinD"] + ["highD"]
evaluated_vehicles_highD = 1000
evaluated_vehicles_exiD = 24
evaluated_vehicles_sinD = 332

# Define positions
x_positions = np.arange(len(partial_costs))
bar_width = 0.6

# Define dataset colors
colors = {
    "highD": "lightgray",
    "exiD": "lightblue",
    "sinD": "lightgreen"
}
bar_colors = [colors[dataset] for dataset in datasets]

# Create the figure and axes
plt.figure(figsize=(12, 8))

# Plot bars
bars = plt.bar(x_positions, compliance_values, color=bar_colors, edgecolor="black", width=bar_width)

# Add percentage annotations on the bars
# for bar, percentage in zip(bars, compliance_values):
#     plt.text(
#         bar.get_x() + bar.get_width() / 2,
#         bar.get_height() + 1,
#         f"{percentage:.1f}%",
#         ha="center",
#         va="bottom",
#         fontsize=12,
#         weight="bold",
#         color="black"
#     )

# Dashed horizontal lines for number of evaluated vehicles
# plt.axhline(y=100, color="black", linestyle="--", linewidth=1.2, label="Baseline (100%)")
# plt.axhline(y=evaluated_vehicles_highD / 10, color="gray", linestyle="--", linewidth=1.2, label="highD")
# plt.axhline(y=evaluated_vehicles_exiD / 10, color="blue", linestyle="--", linewidth=1.2, label="exiD")
# plt.axhline(y=evaluated_vehicles_sinD / 10, color="green", linestyle="--", linewidth=1.2, label="sinD")

# Customization
plt.xticks(x_positions, partial_costs, rotation=45, fontsize=12)
plt.yticks(fontsize=12)
plt.xlabel("Partial Cost Function ID", fontsize=14)
plt.ylabel("Compliance Percentage (%)", fontsize=14)


# Add legend for bar colors
legend_handles = [plt.Rectangle((0,0),1,1, color=colors[dataset], edgecolor="black") for dataset in ["highD", "exiD", "sinD"]]
legend_labels = ["highD", "exiD", "sinD"]
plt.legend(legend_handles, legend_labels, loc="upper right", fontsize=12, title="Datasets")

# Add grid for better readability
plt.grid(axis="y", linestyle="--", alpha=0.7)

# Adjust layout
#plt.tight_layout()

# Show the plot
plt.show()
