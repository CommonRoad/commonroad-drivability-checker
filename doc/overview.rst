.. _overview:

#########
Tutorials
#########

The following code examples are available as Jupyter Notebooks in tutorials/. To execute the tutorials, run following commands in your terminal:

#. Open your console in the root folder of the CommonRoad Drivability Checker.

#. Activate your environment with 

  .. code-block:: bash

    $ conda activate commonroad-py36
    $ cd tutorials/
    $ jupyter notebook name_of_notebook

  **Note that you have to replace** 
    - commonroad-py36 with the name of your Anaconda environment.
    - name_of_notebook, e.g., with 01_python_wrapper.ipynb.

The **basic tutorial** (**Tutorial 00: Drivability Checker - Getting Started**) contains the most important
information to get started with our tool. For a more in-depth demonstration of the different features, see our
**advanced tutorials** (**Tutorial 01 - 07**).

.. toctree::
    :includehidden:

    00_getting_started.nblink
    01_python_wrapper.nblink
    02_commonroad_interface.nblink
    03_road_compliance_checking.nblink
    04_feasibility_checker.nblink
    05_collision_checks_dynamic_obstacles.nblink
    06_curvilinear_coordinate_system.nblink
    07_cost_functions.nblink
