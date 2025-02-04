# import matplotlib.pyplot as plt
# import numpy as np

# # Define road boundaries
# lane_left = np.array([[0, 5], [100, 5]])
# lane_right = np.array([[0, -5], [100, -5]])

# # Define ego trajectory
# trajectory_x = np.linspace(0, 100, 50)
# trajectory_y = 2 * np.sin(trajectory_x / 20)  # Example trajectory

# # Define ego vehicle shape (rectangle dimensions)
# ego_vehicle_length = 3
# ego_vehicle_width = 1.5

# # Function to plot a rectangle for a vehicle
# def plot_vehicle(x, y, length, width, color, label="", fill=False):
#     """Plots a rectangular representation of a vehicle."""
#     rectangle_x = [x - length / 2, x + length / 2, x + length / 2, x - length / 2, x - length / 2]
#     rectangle_y = [y - width / 2, y - width / 2, y + width / 2, y + width / 2, y - width / 2]
    
#     if fill:
#         plt.fill(rectangle_x, rectangle_y, color=color, alpha=0.5, label=label)
#     else:
#         plt.plot(rectangle_x, rectangle_y, color, label=label, linewidth=2)

# # Define obstacles (other vehicles)
# obstacles = [
#     {'x': 30, 'y': 0, 'length': 3, 'width': 1.5, 'color': 'blue'},
#     {'x': 50, 'y': 3, 'length': 3, 'width': 1.5, 'color': 'blue'},
#     {'x': 70, 'y': -3, 'length': 3, 'width': 1.5, 'color': 'blue'}
# ]

# # Create figure
# plt.figure(figsize=(12, 8))

# # Plot road boundaries
# plt.plot(lane_left[:, 0], lane_left[:, 1], 'k--', linewidth=4)
# plt.text(lane_left[-1, 0], lane_left[-1, 1], 'Lane Boundary', verticalalignment='bottom', fontsize=12)
# plt.plot(lane_right[:, 0], lane_right[:, 1], 'k--', linewidth=4)
# plt.text(lane_right[-1, 0], lane_right[-1, 1], 'Lane Boundary', verticalalignment='top', fontsize=12)

# # Plot ego trajectory
# plt.plot(trajectory_x, trajectory_y, 'g-', linewidth=2, label='Ego Trajectory')
# plt.text(trajectory_x[len(trajectory_x)//2], trajectory_y[len(trajectory_y)//2], 
#          'Ego Trajectory', color='blue', fontsize=12)

# # Plot ego vehicle along the trajectory
# for i in range(0, len(trajectory_x), len(trajectory_x) // 10):
#     plot_vehicle(trajectory_x[i], trajectory_y[i], ego_vehicle_length, ego_vehicle_width, 
#                  'limegreen', label='Ego Vehicle' if i == 0 else "")

# # Plot obstacles with shading
# for obs in obstacles:
#     plot_vehicle(obs['x'], obs['y'], obs['length'], obs['width'], obs['color'], 
#                  label='Obstacle' if obs == obstacles[0] else "", fill=True)

# # Highlight start and goal regions
# start_region_radius = 2
# goal_region_radius = 2

# # Start region
# plt.fill_betweenx(
#     [trajectory_y[0] - start_region_radius, trajectory_y[0] + start_region_radius],
#     trajectory_x[0] - start_region_radius, trajectory_x[0] + start_region_radius, 
#     color='green', alpha=0.3
# )
# plt.text(trajectory_x[0], trajectory_y[0], r'$t = t_0$', color='black', 
#          verticalalignment='bottom', fontsize=14)

# # Goal region
# plt.fill_betweenx(
#     [trajectory_y[-1] - goal_region_radius, trajectory_y[-1] + goal_region_radius],
#     trajectory_x[-1] - goal_region_radius, trajectory_x[-1] + goal_region_radius, 
#     color='yellow', alpha=0.3
# )
# plt.text(trajectory_x[-1], trajectory_y[-1], r'$t = t_f$', color='black', 
#          verticalalignment='top', fontsize=14)

# # Add labels, grid, and adjust plot aesthetics
# plt.xlabel('Longitudinal Distance', fontsize=14)
# plt.ylabel('Lateral Distance', fontsize=14)
# plt.grid(True)
# plt.gca().set_aspect('equal', adjustable='box')
# plt.xlim([0, 100])
# plt.ylim([-5.2, 5.2])

# # Tight layout and legend
# plt.tight_layout()
# #plt.legend(loc='upper right')
# plt.show()





# import matplotlib.pyplot as plt
# from matplotlib.patches import FancyArrowPatch

# def draw_box(ax, text, center, width=3, height=6, color="lightblue"):
#     """Draws a single boxed element."""
#     x = center[0] - width / 2
#     y = center[1] - height / 2
#     rect = plt.Rectangle((x, y), width, height, edgecolor="black", facecolor=color, lw=2, zorder=2)
#     ax.add_patch(rect)
#     ax.text(
#         center[0],
#         center[1],
#         text,
#         ha="center",
#         va="center",
#         fontsize=24,
#         weight="bold",
#         zorder=3
#     )

# def draw_arrow(ax, start, end):
#     """Draws a straight arrow between two points."""
#     arrow = FancyArrowPatch(
#         start, end, arrowstyle="->", mutation_scale=12, lw=2, color="black", zorder=1
#     )
#     ax.add_patch(arrow)

# # Set up the figure and axes
# fig, ax = plt.subplots(figsize=(12, 8))
# ax.set_xlim(0, 14)
# ax.set_ylim(0, 10)
# ax.axis("off")

# # Number of cost functions
# cost_labels = ["$J_{SD}$", "$J_{UB}$", "$\\vdots$", "$J_r$"]
# weight_labels = ["$w_{SD}$", "$w_{UB}$", "$\\vdots$", "$w_r$"]

# # Draw cost function labels and arrows to weights
# for i, (c_label, w_label) in enumerate(zip(cost_labels, weight_labels)):
#     ax.text(
#         1, 8 - i * 2, c_label, fontsize=24, ha="center", va="center"
#     )
#     ax.text(
#         6, 8 - i * 2, w_label, fontsize=24, ha="center", va="center"
#     )
#     draw_arrow(ax, (1.5, 8 - i * 2), (5.5, 8 - i * 2))

# # Draw overall cost function box
# overall_center = (10, 5)
# draw_box(ax, "$J_{TR2}$", center=overall_center, width=3, height=6, color="lightblue")

# # Draw arrows from weights to overall cost function
# for i in range(len(cost_labels)):
#     draw_arrow(ax, (6.5, 8 - i * 2), (8.5, 8 - i * 2))

# # Adding a title to the diagram

# # Display the figure
# plt.tight_layout()
# plt.show()

# import matplotlib.pyplot as plt
# from matplotlib.patches import FancyArrowPatch

# def draw_box(ax, text, center, width=5, height=2, color="lightblue"):
#     """Draws a labeled rectangular box."""
#     x = center[0] - width / 2
#     y = center[1] - height / 2
#     rect = plt.Rectangle((x, y), width, height, edgecolor="black", facecolor=color, lw=2)
#     ax.add_patch(rect)
#     ax.text(
#         center[0], center[1], text,
#         ha="center", va="center", fontsize=14, weight="bold"
#     )

# def draw_arrow(ax, start, end, label=None):
#     """Draws an arrow between two points."""
#     arrow = FancyArrowPatch(
#         start, end, arrowstyle="->", mutation_scale=15, lw=2, color="black"
#     )
#     ax.add_patch(arrow)
#     if label:
#         ax.text(
#             (start[0] + end[0]) / 2,
#             start[1] + 0.3,  # Adjusted for proportional placement
#             label,
#             fontsize=14,
#             ha="center",
#             va="center"
#         )

# # Set up the figure
# fig, ax = plt.subplots(figsize=(10, 5))
# ax.set_xlim(0, 12)
# ax.set_ylim(0, 5)
# ax.axis("off")

# # STL Monitor box
# monitor_center = (6, 2.5)
# draw_box(ax, "STL Monitor", center=monitor_center, width=4, height=2, color="lightblue")

# # Arrows connecting inputs and outputs
# # Trajectory input to STL Monitor
# draw_arrow(ax, (1, 3.5), (4, 3.5), label="Trajectory")

# # Traffic Rule input to STL Monitor
# draw_arrow(ax, (1, 1.5), (4, 1.5), label="Formalized Traffic Rule (STL)")

# # STL Monitor to Robustness Values
# draw_arrow(ax, (8, 2.5), (11, 2.5), label="Trajectory Robustness")

# # Adding a title to the diagram

# # Display the figure
# plt.tight_layout()
# plt.show()


import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

def draw_box(ax, text, center, width=3, height=1, color="lightblue"):
    """Draws a labeled rectangular box."""
    x = center[0] - width / 2
    y = center[1] - height / 2
    rect = plt.Rectangle((x, y), width, height, edgecolor="black", facecolor=color, lw=2)
    ax.add_patch(rect)
    ax.text(
        center[0], center[1], text,
        ha="center", va="center", fontsize=12, weight="bold"
    )

def draw_arrow(ax, start, end, label=None):
    """Draws a straight arrow between two points and optionally adds a label."""
    arrow = FancyArrowPatch(
        start, end, arrowstyle="->", mutation_scale=15, lw=2, color="black"
    )
    ax.add_patch(arrow)
    if label:
        ax.text(
            (start[0] + end[0]) / 2,
            start[1] + 0.2,  # Position the label slightly above the arrow
            label,
            ha="center", va="center", fontsize=12, weight="bold"
        )

def draw_text(ax, text, position, fontsize=14):
    """Draws text at a given position."""
    ax.text(
        position[0], position[1], text,
        ha="center", va="center", fontsize=fontsize, weight="bold"
    )

# Set up the figure
fig, ax = plt.subplots(figsize=(12, 4))
ax.set_xlim(0, 14)
ax.set_ylim(0, 4)
ax.axis("off")

# Add components
# \rho(t)
#draw_text(ax, r"$\rho(t)$", position=(1, 2), fontsize=14)
draw_text(ax, "Trajectory Rob.", position=(0.0, 2), fontsize=14)

draw_arrow(ax, (1.1, 2), (2, 2))

# Violation Robustness Extraction
draw_box(ax, "-ve Robustness\nExtraction", center=(4.0, 2), width=4, height=2, color="lightblue")
draw_arrow(ax, (6, 2), (7.0, 2), label=r"$-\rho(t)$")

# Aggregation
draw_box(ax, "Aggregation", center=(9, 2), width=4, height=2, color="lightblue")
draw_arrow(ax, (11, 2), (12, 2))

# J_r
draw_text(ax, r"$J_r$", position=(12.5, 2), fontsize=20)

# Adding a title to the diagram

# Display the figure
plt.tight_layout()
plt.show()



# import matplotlib.pyplot as plt
# from matplotlib.patches import FancyArrowPatch
# import numpy as np

# def draw_box(ax, text, center, width=3, height=1, color="lightblue"):
#     """Draws a labeled rectangular box."""
#     x = center[0] - width / 2
#     y = center[1] - height / 2
#     rect = plt.Rectangle((x, y), width, height, edgecolor="black", facecolor=color, lw=2)
#     ax.add_patch(rect)
#     ax.text(
#         center[0], center[1], text,
#         ha="center", va="center", fontsize=12, weight="bold"
#     )

# def draw_arrow(ax, start, end, label=None):
#     """Draws a straight arrow between two points and optionally adds a label."""
#     arrow = FancyArrowPatch(
#         start, end, arrowstyle="->", mutation_scale=15, lw=2, color="black"
#     )
#     ax.add_patch(arrow)
#     if label:
#         ax.text(
#             (start[0] + end[0]) / 2,
#             start[1] + 0.2,  # Position the label slightly above the arrow
#             label,
#             ha="center", va="center", fontsize=12, weight="bold"
#         )

# def draw_text(ax, text, position, fontsize=14):
#     """Draws text at a given position."""
#     ax.text(
#         position[0], position[1], text,
#         ha="center", va="center", fontsize=fontsize, weight="bold"
#     )

# def plot_road(ax):
#     """Plots a simple road layout."""
#     # Lane boundaries
#     lane_left = np.array([[0, 2], [10, 2]])
#     lane_right = np.array([[0, -2], [10, -2]])

#     ax.plot(lane_left[:, 0], lane_left[:, 1], 'k--', linewidth=3)
#     ax.plot(lane_right[:, 0], lane_right[:, 1], 'k--', linewidth=3)

# def plot_vehicles_with_distance(ax):
#     """Plots the ego and preceding vehicles with a label for the longitudinal distance."""
#     # Ego vehicle
#     ego_x, ego_y = 2, 0
#     ax.add_patch(plt.Rectangle((ego_x - 0.5, ego_y - 0.25), 2, 1.5, color="green"))
#     ax.text(ego_x+0.3, ego_y + 0.4, "Ego Vehicle", fontsize=15, ha="center", va="bottom", #color = "white"
#            weight = "bold" )

#     # Preceding vehicle
#     lead_x, lead_y = 6, 0
#     ax.add_patch(plt.Rectangle((lead_x - 0.5, lead_y - 0.25), 2, 1.5, color="lightblue"))
#     ax.text(lead_x + 0.3, lead_y + 0.4, "Other Vehicle", fontsize=15, ha="center", va="bottom", #color = "white"
#             weight = "bold" )

#     # Longitudinal distance label
#     ax.annotate(
#         "$safe\_distance$", 
#         xy=(3.55, 0.5), xycoords='data',
#         xytext=(4.9, 0.44), textcoords='data',  # Adjusting text position to be above the arrow
#         arrowprops=dict(arrowstyle="<->", color="black", lw=1.5),
#         fontsize=14, ha="center", va="bottom"  # Position the label above the arrow
#     )

# # Set up the figure
# fig, ax = plt.subplots(figsize=(14, 6))
# ax.set_xlim(0, 10)
# ax.set_ylim(-2.3, 2.2)
# ax.set_ylabel("Lateral Distance", fontsize=14)
# ax.set_xlabel("Longitudinal Distance", fontsize=14)
# # Plot the road network
# plot_road(ax)

# # Plot vehicles with longitudinal distance label
# plot_vehicles_with_distance(ax)

# # Adding a title to the diagram

# # Display the figure
# plt.tight_layout()
# plt.show()
