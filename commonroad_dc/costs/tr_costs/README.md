# Traffic Rule Based Partial Cost Functions

## Overview

This repository contains the implementation of partial cost functions designed to capture the semantics of traffic rules based on the robustness scores from the [CommonRoad STL monitor](https://gitlab.lrz.de/cps/commonroad/commonroad-stl-monitor). Our work leverages the German "[Bundeseinheitlicher Tatbestandskatalog](https://www.kba.de/DE/Themen/ZentraleRegister/FAER/BT_KAT_OWI/bkat_owi_22_08_2024.pdf;jsessionid=861483D30B1A928AD641DAA0A78BB1C6.live21303?__blob=publicationFile&v=4)" to assign weights to the partial cost functions. These partial cost functions and their corresponding weights are integrated into the **[TR2]** overall cost function in [evaluation.py](https://gitlab.lrz.de/cps/commonroad-drivability-checker/-/blob/local_development/commonroad_dc/costs/evaluation.py).


## Structure

- **tr_cost/**
  - **tr_evaluator.py**: The STL Rule Monitoring framework embedded into the `TrafficRuleEvaluator`.
  - **tr_partial_cost_functions.py**: All partial cost functions (e.g., safe distance, speed limits).
  - **utilities.py**: Helper functions (e.g., robustness rescaling, vehicle object creation).
  - **config.yaml**: Configuration for the road network and critical parameters.

  - **tests_and_evaluation/**
    - **datasets_and_results/** : exiD, highD, and sinD datasets and evaluation results
    - **test_rules/** : Unittests for the partial cost functions.


## Prerequisites

- [commonroad_io](https://gitlab.lrz.de/cps/commonroad/commonroad-io)
- [CommonRoad stl monitor](https://gitlab.lrz.de/cps/commonroad/commonroad-stl-monitor)
- Python 3.8+


## Key References

- Althoff et al., 2017
- Maierhofer et al., 2020
- Gressenbuch et al., 2021
- Maierhofer et al., 2022

