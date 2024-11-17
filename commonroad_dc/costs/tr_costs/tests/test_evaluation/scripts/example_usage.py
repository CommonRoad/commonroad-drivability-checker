# example usage

import os
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import CommonRoadSolutionReader
from commonroad_dc.costs.evaluation import CostFunctionEvaluator
import imageio
import xml.etree.ElementTree as ET
from pathlib import Path
from SMP.motion_planner.utility import visualize_solution
import shutil



scenario_file_path = Path(__file__).parent.parent.joinpath(
    
                                                    
    'scenarios/DEU_Guetersloh-39_5_T-1.xml'
    #'scenarios/BEL_Putte-11_1_T-1.xml'
    #'scenarios/DEU_Flensburg-31_1_T-1.xml'
    #'scenarios/DEU_BadEssen-4_1_T-1.xml'
    #'scenarios/DEU_Meckenheim-2_3_T-1.xml'
    #'scenarios/DEU_Ibbenbueren-13_3_T-1.xml'
    #'scenarios/ARG_Carcarana-11_1_T-1.xml'
    
)


solution_file_path = Path(__file__).parent.parent.joinpath( 
                                                       
    # # Guetersloh
    'solutions/KS2:SA1:DEU_Guetersloh-39_5_T-1:2020a/ce2bd15f-08ee-4996-8948-3e535393f4d1.xml'
    #'solutions/KS2:SM1:DEU_Guetersloh-39_5_T-1:2020a/0da67679-f6d4-4fc0-ab9c-00d0fd00f9e3.xml'
    #'solutions/KS2:SM1:DEU_Guetersloh-39_5_T-1:2020a/0c745c5d-2666-408c-b8b0-8e312e84be36.xml'
    #'solutions/KS2:SM1:DEU_Guetersloh-39_5_T-1:2020a/0d69aa7c-33a3-4b43-9022-b4bceeb44969.xml'
    #'solutions/KS2:SM1:DEU_Guetersloh-39_5_T-1:2020a/01a44f8b-fadd-4032-b3e3-82454d0409ad.xml'
    
    # # Putte
    #'solutions/KS2:SM1:BEL_Putte-11_1_T-1:2020a/2d4c9890-34f8-4525-bb2b-dfb648d493f0.xml'
    #'solutions/KS2:SM1:BEL_Putte-11_1_T-1:2020a/2d655883-b373-4896-85ba-ca316a861b7c.xml'
    #'solutions/KS2:SM1:BEL_Putte-11_1_T-1:2020a/2f35f9c3-aeaf-45f2-ab1b-159d6e612e62.xml'
    #'solutions/KS2:SM1:BEL_Putte-11_1_T-1:2020a/3e00d681-f990-4947-93ac-143bea9388af.xml' # problem resolved
    
    # # Flensburg
    #'solutions/KS2:SM1:DEU_Flensburg-31_1_T-1:2020a/0c308d9c-c809-4f21-9be2-367bb7b42919.xml' # Problem resolved
    #'solutions/KS2:SM1:DEU_Flensburg-31_1_T-1:2020a/0ca4d11a-9686-4c88-9310-7ab16f7efbf6.xml'
    #'solutions/KS2:SM1:DEU_Flensburg-31_1_T-1:2020a/0ca6fd68-656b-493a-a7bb-95604e63eead.xml'
    #'solutions/KS2:SM1:DEU_Flensburg-31_1_T-1:2020a/0cdc0601-1b03-4bec-882c-2ca603c12a47.xml'
    
    # # BadEssen
    #'solutions/KS2:SM3:DEU_BadEssen-4_1_T-1:2020a/6e12386a-4331-489a-9860-fd6aac851eac.xml'
    #'solutions/KS2:SM3:DEU_BadEssen-4_1_T-1:2020a/98b9d941-2cf5-40b7-a0dc-7c5ef21db75a.xml'
    #'solutions/KS2:SM3:DEU_BadEssen-4_1_T-1:2020a/b27c81a0-4b7a-4cb6-b650-98a3599c9579.xml'
    #'solutions/KS2:SM3:DEU_BadEssen-4_1_T-1:2020a/94f6b3a5-06f5-49ad-bc98-e8d7facc4add.xml'
    
    ## Meckenheim
    #'solutions/KS2:SM1:DEU_Meckenheim-2_3_T-1:2020a/0a2a8392-b943-4c57-b60d-49edd554992e.xml'
    #'solutions/KS2:SM1:DEU_Meckenheim-2_3_T-1:2020a/3b521e2d-62b2-4c61-a081-5bd153ec5834.xml' # best solution
    #'solutions/KS2:SM1:DEU_Meckenheim-2_3_T-1:2020a/3cfcbcd4-8425-4076-993f-4c926919886a.xml'
    
    ## Carcarana
    #'solutions/KS2:SA1:ARG_Carcarana-11_1_T-1:2020a/4f12ef7b-f723-467b-9893-4de2ea972b30.xml'
    
) 

scenario, planning_problem_set = CommonRoadFileReader(scenario_file_path).open(lanelet_assignment=True)
solution = CommonRoadSolutionReader.open(solution_file_path)

# evaluate solution
ce = CostFunctionEvaluator.init_from_solution(solution)
cost_result = ce.evaluate_solution(scenario, planning_problem_set, solution)

print(cost_result)




#------------------ visualizing the solution  ----------- # Can be deactived if not needed!

# tree = ET.parse(solution_file_path)
# root = tree.getroot()
# trajectory_node = root.find("ksTrajectory")

# _, trajectory = CommonRoadSolutionReader._parse_trajectory(
#     trajectory_node=trajectory_node
# )

# image_dir = 'solution_images'
# animation_dir = 'animations'
# os.makedirs(image_dir, exist_ok=True)
# os.makedirs(animation_dir, exist_ok=True)

# # Counter to keep track of image files
# image_counter = 0


# # Redefine plt.show to save images instead of showing them
# import matplotlib.pyplot as plt
# original_show = plt.show

# def save_fig_as_image():
#     global image_counter
#     image_path = os.path.join(image_dir, f"frame_{image_counter:04d}.png")
#     plt.savefig(image_path)
#     image_counter += 1
#     plt.close()

# plt.show = save_fig_as_image

# # Call the visualize_solution function
# # Ensure `trajectory` is defined appropriately for your case
# visualize_solution(scenario, planning_problem_set, trajectory)

# # Restore original plt.show function
# plt.show = original_show

# # Create GIF from saved images
# images = []
# for i in range(image_counter):
#     image_path = os.path.join(image_dir, f"frame_{i:04d}.png")
#     images.append(imageio.imread(image_path))
# # Save as GIF in the animations directory
# gif_path = Path(__file__).parent.joinpath('z_solution_animation.gif')
# imageio.mimsave(gif_path, images, fps=5)
# print(f"Animation saved to {gif_path}")
# shutil.rmtree(image_dir)

