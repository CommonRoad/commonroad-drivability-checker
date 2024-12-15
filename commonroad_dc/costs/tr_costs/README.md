# Traffic Rule Based Partial Cost Functions

## Overview

This repository contains the implementation of a framework for evaluating traffic rule compliance using Signal Temporal Logic (STL) and designing cost functions for motion planner evaluation in CommonRoad.

## Structure

- **tr_cost/**
  - **tr_evaluator.py**: Contains the STL Rule Monitoring framework housed in the TrafficRuleEvaluator.
  - **tr_partial_cost_functions.py**: Implements all partial cost functions (e.g., safe distance, speed limits).
  - **utilities.py**: Includes helper functions (e.g., robustness rescaling, vehicle object creation).
  - **config.yaml**: Contains the YAML configuration for the road network and critical parameters.

  - **tests/**
    - **evaluation/**
      - **scenarios/**: Scenario files for evaluation.
      - **solutions/**: Solution files for evaluation.
      - **scripts/**
        - **modify_benchmark.py**: For batch modification of cost function IDs of solutions.
        - **run_evaluation.py**: For evaluation.
        - **evaluation_results.yaml**: Output of the evaluation.
        - **example_usage.py**: A minimal example.
        - **z_solution_animation.gif**: Solution visualization example.
        
    - **test_rules/**
      - **scenarios/**: Rule-specific hand-crafted scenarios.
      - **scripts/**
        - **test_traffic_costs.py**: Validate the partial cost functions on corresponding rules.
        - **visualize_test_scenarios.py**: For visual monitoring of vehicle behaviors.
        - **z_test_scenario_animation.gif**: Output of `visualize_test_scenarios.py`.

## Integration

Partial cost functions are incorporated into the **[TR2]** overall cost function in `evaluation.py`. See [CommonRoad Drivability Checker](https://gitlab.lrz.de/cps/commonroad-drivability-checker/-/blob/local_development/commonroad_dc/costs/evaluation.py).

This framework leverages the German "Bundeseinheitlicher Tatbestandskatalog" (BKat) to assign weights to the partial cost functions.

## Prerequisites

- `commonroad_io`
- `commonroad-stl-monitor`
- Python 3.8+

## Usage

See [Example Usage](https://gitlab.lrz.de/cps/commonroad-drivability-checker/-/blob/local_development/commonroad_dc/costs/tr_costs/tests/evaluation/scripts/example_usage.py).

## Key References

- Althoff et al., 2017
- Maierhofer et al., 2020
- Gressenbuch et al., 2021
- Maierhofer et al., 2022
- CommonRoad Benchmark Documentation
