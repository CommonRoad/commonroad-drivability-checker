# file for batch evaluation

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import CommonRoadSolutionReader
from commonroad_dc.costs.evaluation import CostFunctionEvaluator
import traceback
import json
import yaml
from pathlib import Path

# scenario path
scenario_file_path = Path(__file__).parent.parent.joinpath('scenarios',
    
    #'DEU_Guetersloh-39_5_T-1.xml' # can be used for intersection
    #'BEL_Putte-11_1_T-1.xml'
    'DEU_Flensburg-31_1_T-1.xml'
    #'DEU_BadEssen-4_1_T-1.xml'   
    #'DEU_Meckenheim-2_3_T-1.xml'  # Takes too much time
    #'DEU_Ibbenbueren-13_3_T-1.xml'
    #'DEU_Muc-4_2_T-1.xml'
    #'ESP_Monzon-6_1_T-1.xml'
    #'DEU_BadWaldsee-4_5_T-1.xml'
    #'DEU_Hennigsdorf-5_2_T-1.xml'
    #'BEL_Wervik-4_1_T-1.xml'
    #'ARG_Carcarana-11_1_T-1.xml'
    
)

# solution directory
solutions_directory = Path(__file__).parent.parent.joinpath('solutions',
    
    
    ### Guetersloh
    #'KS2:SA1:DEU_Guetersloh-39_5_T-1:2020a'
    #'KS2:SM3:DEU_Guetersloh-39_5_T-1:2020a'
    #'KS2:SM1:DEU_Guetersloh-39_5_T-1:2020a'
    
    #'KS2:SA1:ARG_Carcarana-11_1_T-1:2020a'
    #'KS2:SM1:ARG_Carcarana-11_1_T-1:2020a'
    #'KS2:SM3:ARG_Carcarana-11_1_T-1:2020a'

    #'KS2:SM1:BEL_Putte-11_1_T-1:2020a'
    
    #'KS2:SM1:BEL_Wervik-4_1_T-1:2020a'
    #'KS2:SM3:BEL_Wervik-4_1_T-1:2020a'
    #'KS2:SA1:BEL_Wervik-4_1_T-1:2020a'

    #'KS2:SM1:DEU_BadEssen-4_1_T-1:2020a'
    #'KS2:SM3:DEU_BadEssen-4_1_T-1:2020a'
    #'KS2:SA1:DEU_BadEssen-4_1_T-1:2020a'
    
    #'KS2:SA1:DEU_BadWaldsee-4_5_T-1:2020a'
    #'KS2:SM1:DEU_BadWaldsee-4_5_T-1:2020a'
    #'KS2:SM3:DEU_BadWaldsee-4_5_T-1:2020a'

    #'KS2:SM1:DEU_Flensburg-31_1_T-1:2020a'
    'KS2:SM3:DEU_Flensburg-31_1_T-1:2020a'
    #'KS2:SA1:DEU_Flensburg-31_1_T-1:2020a'
    
    #'KS2:SM1:DEU_Hennigsdorf-5_2_T-1:2020a'
    
    #'KS2:SM1:DEU_Ibbenbueren-13_3_T-1:2020a'
    #'KS2:SM3:DEU_Ibbenbueren-13_3_T-1:2020a'
    #'KS2:SA1:DEU_Ibbenbueren-13_3_T-1:2020a' 
    
    #'KS2:SM1:DEU_Meckenheim-2_3_T-1:2020a'
    #'KS2:SM1:DEU_Muc-4_2_T-1:2020a'
    
    #'KS2:SM3:ESP_Monzon-6_1_T-1:2020a'
    #'KS2:SM1:ESP_Monzon-6_1_T-1:2020a'

)

scenario, planning_problem_set = CommonRoadFileReader(scenario_file_path).open(lanelet_assignment=True)


def convert_numpy_to_python(data):
    """Recursively convert numpy objects to native Python types."""
    if isinstance(data, dict):
        return {key: convert_numpy_to_python(value) for key, value in data.items()}
    elif isinstance(data, list):
        return [convert_numpy_to_python(item) for item in data]
    elif hasattr(data, "item"):  # Handles numpy scalars
        return data.item()
    return data

# Define the output file path for YAML
output_file = Path(__file__).parent.joinpath('evaluation_results.yaml')
results = []

# Add the scenario information as the file header
file_header = {
    "scenario": scenario_file_path.name,
    "solutions": []
}
results.append(file_header)

# Evaluation loop
counter = 0
max_iterations = 5 # Max number of files to evaluate
for solution_file in solutions_directory.glob("*.xml"):
    if counter >= max_iterations:
        break
    try:
        # Load solution file
        solution = CommonRoadSolutionReader.open(solution_file)
        
        # Initialize cost function evaluator
        ce = CostFunctionEvaluator.init_from_solution(solution)

        # Evaluate the solution
        cost_result = ce.evaluate_solution(scenario, planning_problem_set, solution)
        pp_result = next(iter(cost_result.pp_results.values()))  # First planning problem

        # Extract partial costs
        partial_costs = {
            str(key.name): value
            for key, value in pp_result.partial_costs.items()
        }

        # Dynamically sort partial costs alphabetically (optional, based on your needs)
        ordered_partial_costs = dict(sorted(partial_costs.items()))

        # Append solution results under the scenario
        file_header["solutions"].append({
            "solution_file": solution_file.name,
            "partial_costs": convert_numpy_to_python(ordered_partial_costs),
            "total_cost": convert_numpy_to_python(cost_result.total_costs),
        })

        counter += 1  # Increment the counter

    except Exception as e:
        print(f"Error evaluating {solution_file.name}: {e}")
        traceback.print_exc()
        file_header["solutions"].append({
            "solution_file": solution_file.name,
            "error": str(e)
        })

# Save results to a YAML file
with open(output_file, "w") as f:
    yaml.dump(results, f, default_flow_style=False, sort_keys=False)

print(f"Evaluation complete. Results saved to {output_file}")