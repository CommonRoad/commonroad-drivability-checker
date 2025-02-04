# Traffic Rule Based Partial Cost Functions

## Overview

This repository contains the implementation of partial cost functions designed to capture the semantics of traffic rules based on the robustness scores from the [commonroad stl monitor](https://gitlab.lrz.de/cps/commonroad/commonroad-stl-monitor). Our work leverages the German "[Bundeseinheitlicher Tatbestandskatalog](https://www.kba.de/DE/Themen/ZentraleRegister/FAER/BT_KAT_OWI/bkat_owi_22_08_2024.pdf;jsessionid=861483D30B1A928AD641DAA0A78BB1C6.live21303?__blob=publicationFile&v=4)" to assign weights to the partial cost functions. These partial cost functions and their corresponding weights compose the overall cost function **[TR2]** in [evaluation.py](https://gitlab.lrz.de/cps/commonroad-drivability-checker/-/blob/local_development/commonroad_dc/costs/evaluation.py).


## Structure
- **config.yaml**: Configuration for the road network and nominal robustness.
- **tr_costs/**
  - **tr_evaluator.py**: The STL Rule Monitoring framework embedded into the class `TrafficRuleEvaluator`.
  - **tr_partial_cost_functions.py**: Traffic-rule based partial cost functions (e.g., safe distance, speed limits).
  - **utilities.py**: Helper functions (e.g., vehicle object creation).
  - **datasets_and_results/** : exiD, highD, and sinD datasets and evaluation results.

## Prerequisites

- [commonroad_io](https://gitlab.lrz.de/cps/commonroad/commonroad-io)
- [commonroad stl monitor](https://gitlab.lrz.de/cps/commonroad/commonroad-stl-monitor)
- Python 3.8+


## Key References

- M. Althoff, M. Koschi, and S. Manzinger. “CommonRoad: Composable Bench-
  marks for Motion Planning on Roads.” In: Proc. of the IEEE Intelligent Vehicles
  Symposium. 2017, pp. 719–726.

- S. Maierhofer, A.-K. Rettinger, E. C. Mayer, and M. Althoff. “Formalization of
  Interstate Traffic Rules in Temporal Logic.” In: Proc. of the IEEE Intelligent Vehicles
  Symposium (IV). 2020, pp. 752–759.

- L. Gressenbuch and M. Althoff. “Predictive Monitoring of Traffic Rules.” In: Proc.
  of the IEEE International Intelligent Transportation Systems Conference (ITSC). 2021,
  pp. 915–922.

- S. Maierhofer, P. Moosbrugger, and M. Althoff. “Formalization of Intersection
  Traffic Rules in Temporal Logic.” In: Proc. of the IEEE Intelligent Vehicles Symposium
  (IV). 2022, pp. 1135–1144.

