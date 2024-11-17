# file for batch evaluation

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import CommonRoadSolutionReader
from commonroad_dc.costs.evaluation import CostFunctionEvaluator
import traceback
import json
from pathlib import Path

# scenario path
scenario_file_path = Path(__file__).parent.parent.joinpath(
    
    'scenarios/DEU_Guetersloh-39_5_T-1.xml'
    #'scenarios/BEL_Putte-11_1_T-1.xml'
    #'scenarios/DEU_Flensburg-31_1_T-1.xml'
    #'scenarios/DEU_BadEssen-4_1_T-1.xml'   
    #'scenarios/DEU_Meckenheim-2_3_T-1.xml'
    #'scenarios/DEU_Ibbenbueren-13_3_T-1.xml'    
    #'scenarios/ARG_Carcarana-11_1_T-1.xml'
)

# solution directory
solutions_directory = Path(__file__).parent.parent.joinpath(
    
    
    ### Guetersloh
    #'solutions/KS2:SA1:DEU_Guetersloh-39_5_T-1:2020a'
    'solutions/KS2:SM3:DEU_Guetersloh-39_5_T-1:2020a'
    #'solutions/KS2:SM1:DEU_Guetersloh-39_5_T-1:2020a'
    
    #'solutions/KS2:SA1:ARG_Carcarana-11_1_T-1:2020a'
    #'solutions/KS2:SM1:ARG_Carcarana-11_1_T-1:2020a'
    #'solutions/KS2:SM3:ARG_Carcarana-11_1_T-1:2020a'

    #'solutions/KS2:SM1:BEL_Putte-11_1_T-1:2020a'
    
    #'solutions/KS2:SM1:BEL_Wervik-4_1_T-1:2020a'
    #'solutions/KS2:SM3:BEL_Wervik-4_1_T-1:2020a'

    #'solutions/KS2:SM1:DEU_BadEssen-4_1_T-1:2020a'
    #'solutions/KS2:SM3:DEU_BadEssen-4_1_T-1:2020a'

    #'solutions/KS2:SM1:DEU_BadWaldsee-4_5_T-1:2020a'
    #'solutions/KS2:SM3:DEU_BadWaldsee-4_5_T-1:2020a'

    #'solutions/KS2:SM1:DEU_Flensburg-31_1_T-1:2020a'
    #'solutions/KS2:SM3:DEU_Flensburg-31_1_T-1:2020a'
    
    #'solutions/KS2:SM1:DEU_Hennigsdorf-5_2_T-1:2020a'
    #'solutions/KS2:SM1:DEU_Ibbenbueren-13_3_T-1:2020a'
    #'solutions/KS2:SM1:DEU_Meckenheim-2_3_T-1:2020a'
    #'solutions/KS2:SM1:DEU_Muc-4_2_T-1:2020a'
    
    #'solutions/KS2:SM3:ESP_Monzon-6_1_T-1:2020a'
    #'solutions/KS2:SM1:ESP_Monzon-6_1_T-1:2020a'
    
    
    
    #'solutions/KS2:SM3:DEU_Ibbenbueren-13_3_T-1:2020a'

)

output_file = Path(__file__).parent.joinpath('evaluation_results.json')
scenario, planning_problem_set = CommonRoadFileReader(scenario_file_path).open(lanelet_assignment=True)
results = []

counter = 0
max_iterations = 100
for solution_file in solutions_directory.glob("*.xml"):
    if counter >= max_iterations:  # Max of 100 files to be processed, can be changed
        break
    try:
        solution = CommonRoadSolutionReader.open(solution_file)
        
        ce = CostFunctionEvaluator.init_from_solution(solution)

        cost_result = ce.evaluate_solution(scenario, planning_problem_set, solution)

        pp_result = next(iter(cost_result.pp_results.values()))  # the first planning problem

        # Extract only the partial costs
        partial_costs = {
            str(key.name): value
            for key, value in pp_result.partial_costs.items()
        }

        results.append({
            "solution_file": solution_file.name,
            "partial_costs": partial_costs
        })
        counter += 1  

    except Exception as e:
        print(f"Error evaluating {solution_file.name}: {e}")
        traceback.print_exc()
        results.append({
            "solution_file": solution_file.name,
            "error": str(e)
        })

# Save results to a JSON file
with open(output_file, "w") as f:
    json.dump(results, f, indent=4)

print(f"Evaluation complete. Results saved to {output_file}")
