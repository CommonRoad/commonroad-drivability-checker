# file for setting the desired CostFunction in batch

from pathlib import Path
import xml.etree.ElementTree as ET


solutions_directory = Path(__file__).parent.joinpath(
    
    # uncomment folder to modify its benchmark
    
    #'solutions/KS2:SA1:DEU_Guetersloh-39_5_T-1:2020a'
    #'solutions/KS2:SM1:DEU_Guetersloh-39_5_T-1:2020a'
    #'solutions/KS2:SA1:ARG_Carcarana-11_1_T-1:2020a'
    #'solutions/KS2:SM1:ARG_Carcarana-11_1_T-1:2020a'
    #'solutions/KS2:SM1:BEL_Putte-11_1_T-1:2020a'
    #'solutions/KS2:SM1:BEL_Wervik-4_1_T-1:2020a'
    #'solutions/KS2:SM1:DEU_BadEssen-4_1_T-1:2020a'
    #'solutions/KS2:SM1:DEU_BadWaldsee-4_5_T-1:2020a'
    #'solutions/KS2:SM1:DEU_Flensburg-31_1_T-1:2020a'
    #'solutions/KS2:SM1:DEU_Hennigsdorf-5_2_T-1:2020a'
    #'solutions/KS2:SM1:DEU_Ibbenbueren-13_3_T-1:2020a'
    #'solutions/KS2:SM1:DEU_Meckenheim-2_3_T-1:2020a'
    #'solutions/KS2:SM1:DEU_Muc-4_2_T-1:2020a'
    #'solutions/KS2:SM1:ESP_Monzon-6_1_T-1:2020a'
    #'solutions/KS2:SM3:ARG_Carcarana-11_1_T-1:2020a'
    #'solutions/KS2:SM3:BEL_Wervik-4_1_T-1:2020a'
    #'solutions/KS2:SM3:DEU_BadEssen-4_1_T-1:2020a'
    #'solutions/KS2:SM3:DEU_BadWaldsee-4_5_T-1:2020a'
    #'solutions/KS2:SM3:DEU_Flensburg-31_1_T-1:2020a'
    #'solutions/KS2:SM3:DEU_Guetersloh-39_5_T-1:2020a'
    #'solutions/KS2:SM3:DEU_Ibbenbueren-13_3_T-1:2020a'
    'solutions/KS2:SM3:ESP_Monzon-6_1_T-1:2020a'

)

def modify_benchmark_id(file_path : Path, target_cost_function : str):
    """Modify the benchmark_id in the solution file XML."""
    tree = ET.parse(file_path)
    root = tree.getroot()
    if root.tag == "CommonRoadSolution" and "benchmark_id" in root.attrib:
        parts = root.attrib["benchmark_id"].split(":")
        if len(parts) > 1:
            parts[1] = target_cost_function
            root.attrib["benchmark_id"] = ":".join(parts)
        tree.write(file_path)  # Save the modified XML back
        


for solution_file in solutions_directory.glob("*.xml"):
    # replace with desired cost function eg. JB1, SM1, SM2, SM3, TR1 etc.     
    modify_benchmark_id(solution_file, "TR1")