# Evaluation file

import os
import time
from pathlib import Path

import yaml
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import CommonRoadSolutionReader

from commonroad_dc.costs.evaluation import CostFunctionEvaluator


# Function to convert numpy types to native Python types
def convert_numpy_to_python(data):
    if isinstance(data, dict):
        return {key: convert_numpy_to_python(value) for key, value in data.items()}
    elif isinstance(data, list):
        return [convert_numpy_to_python(item) for item in data]
    elif hasattr(data, "item"):  
        return data.item()
    return data


data_directory = Path(__file__).parent.joinpath(
    # datasets
    
    "highD"   # used for general and interstate partial cost functions, except CEV
    #"SinD"    # used for intersection partial cost functios
    #"exiD"     # used for CEV parial function, due to presence of ramps
    )  

output_file = Path(__file__).parent.joinpath(
    # name the yaml file to save the evaluation result in the z_results/
    
    
    "z_results/highD_overall.yaml"
    
     
    )

results = []

xml_files = sorted(data_directory.glob("*.xml"))

# Seperate files
scenario_files = [f for f in xml_files if "cr" in f.name]
solution_files = {f for f in xml_files if "solution" in f.name}

# Match scenarios to solutions
scenario_solution_path = {sce : soln 
                     for sce in scenario_files
                     for soln in solution_files
                     if os.path.splitext(os.path.splitext(sce)[0])[0] == os.path.splitext(os.path.splitext(soln)[0])[0] 
                     } 
counter = 0
max_iterations = 1
start_time = time.time()
for scenario_file, solution_file in scenario_solution_path.items():
    
    if counter >= max_iterations:
        break
    scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open(lanelet_assignment=True)
    scenario_result = {
    "scenario": scenario_file.name,
    "solutions": []  
        }
    try:
        solution = CommonRoadSolutionReader.open(solution_file)
        ce = CostFunctionEvaluator.init_from_solution(solution)
        cost_result = ce.evaluate_solution(scenario, planning_problem_set, solution)
        pp_result = next(iter(cost_result.pp_results.values())) # for non collaborative scenarios
        partial_costs = {
        # weighting can be activated 
        str(partial_cost_function.name): cost_value #* pp_result.weights.get(partial_cost_function) 
        for partial_cost_function, cost_value in pp_result.partial_costs.items()
        } 
        scenario_result["solutions"].append({
        #   "solution_file": solution_file.name,
            "partial_costs": convert_numpy_to_python(partial_costs),
            "total_cost": convert_numpy_to_python(cost_result.total_costs),
        })
        results.append(scenario_result) # Add scenario_result to results
        counter += 1 
    # Will not happen for selected scenarios,
    except Exception as e:
        print(f"Error evaluating {scenario_file.name} : {e}") 
end_time = time.time()
# Save results to YAML file
with open(output_file, "w") as f:
    yaml.dump(results, f, default_flow_style=False, sort_keys=False)
print(f"Evaluation complete. Results saved to {output_file}")
print(f"Elapsed Time is {end_time-start_time:.2f} seconds")

