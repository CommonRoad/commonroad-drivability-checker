
import matplotlib.pyplot as plt
import os
import imageio
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer
import shutil
from pathlib import Path

# Paths for the scenario file and output directories
file_path = Path(__file__).parent.joinpath(
    
    # ----------------------Uncomment the desired test scenario to visualize -------------------------------
    
    #"DEU_test_safe_distance.xml"
    "DEU_test_recapture_safe_distance.xml"
    #"DEU_test_unnecessary_braking.xml"
    #"DEU_test_max_speed_limit.xml"
    #"DEU_test_preserve_traffic_flow.xml"
    #"DEU_test_standstill.xml"
    #"DEU_test_overtaking_right_normal.xml"
    #"DEU_test_overtaking_right_congestion.xml"
    #"DEU_test_overtaking_right_broad_lane_marking.xml"
    #"DEU_test_reversing_and_u_turn.xml"
    #"DEU_test_emergency_three_lanes_with_shoulder.xml"
    #"DEU_test_emergency_two_lanes_not_broad_enough.xml"
    #"DEU_test_consider_entering_vehicles_for_lane_change.xml"
    #"DEU_test_stop_line.xml"
    #"DEU_test_stop_at_red_light.xml"
)

image_dir = 'solution_images'
os.makedirs(image_dir, exist_ok=True)
scenario, planning_problem_set = CommonRoadFileReader(file_path).open()
print(f"VEHICLES IN SCENARIO: {[ob.obstacle_id for ob in scenario.dynamic_obstacles]}")

time_steps = range(0, 50) # Time steps to render

for time_step in time_steps:
    plt.figure(figsize=(25, 10))
    rnd = MPRenderer()
    rnd.draw_params.time_begin = time_step  # Update to current time step
    scenario.draw(rnd)
    rnd.render()
    # Add labels for each dynamic obstacle's ID at the current time step
    for obstacle in scenario.dynamic_obstacles:
        # Check if the obstacle has a state at the current time step
        current_state = next(
            (state for state in obstacle.prediction.trajectory.state_list if state.time_step == time_step),
            None
        )
        if current_state:
            # add text as obstacle_id at the obstacle's position
            plt.text(
                current_state.position[0],  # X-coordinate
                current_state.position[1],  # Y-coordinate
                f'{obstacle.obstacle_id}',  # Text label
                fontsize=15,
                color='black',
                ha='center',
                va='center',
                zorder=25,
                bbox=dict(facecolor='white', alpha=0.6, edgecolor='black', boxstyle='round,pad=0.2')
            )
            
    plt.draw()  # Draw the plot
    
    image_path = os.path.join(image_dir, f"frame_{time_step:04d}.png")
    plt.savefig(image_path)
    plt.close()  # Close the plot to free memory
# Create GIF from saved images
images = []
for i in time_steps:
    image_path = os.path.join(image_dir, f"frame_{i:04d}.png")
    if os.path.exists(image_path):
        images.append(imageio.imread(image_path))
# Save as GIF in the 'tests/' directory
gif_path = Path(__file__).parent.joinpath('z_test_scenario_animation.gif')
imageio.mimsave(gif_path, images, fps=5)
print(f"Animation saved to {gif_path}")
#  Clean up image directory
shutil.rmtree(image_dir)
