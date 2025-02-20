.. CommonRoad Drivability Checker documentation master file

CommonRoad Drivability Checker
==============================

Collision avoidance, kinematic feasibility, and road-compliance must be validated to ensure the drivability of planned motions for autonomous vehicles. These aforementioned checks are unified in the CommonRoad Drivability Checker that is compatible with the CommonRoad benchmark suite and can be used with hundreds of existing scenarios. 
The CommonRoad Drivability Checker consists of five core modules:

- **collision**: The collision checker module checks whether given geometric objects (e.g., rectangles or triangles) collide with each other. Based on the geometric representation, we provide a computationally efficient method to represent complex traffic scenarios for collision checking.

- **boundary**: The road boundary module determines the road compliance of a given trajectory by either checking whether the ego vehicle is still fully enclosed in the road network or collision checking with obstacles that model the boundary of the road network. Our module provides two different approaches (triangulation and the creation of oriented rectangles) to generate road boundary obstacles.

- **feasibility**: The feasibility module builds on top of the vehicle models provided by CommonRoad. It determines the feasibility of a given trajectory by reconstructing the input to the corresponding (non-linear) vehicle model. Trajectories are feasible if the obtained input respects the constraints of the vehicle model, e.g., limited steering rate.

- **costs**: The costs module implements costs functions of the CommonRoad Benchmark. More specific details can be found in the `cost function documentation <https://gitlab.lrz.de/tum-cps/commonroad-cost-functions/-/blob/master/costFunctions_commonRoad.pdf>`_

Getting Started 
===============

Head over to :ref:`Getting Started<gettingStarted>` to learn how to obtain and install the CommonRoad Drivability Checker.
In :ref:`Tutorials <overview>`, you can find jupyter notebooks on how to use the CommonRoad drivability checker.

Overview
========

.. toctree::
   :maxdepth: 2
    
    Getting Started <gettingStarted.rst>
    Tutorials <overview.rst>
    API <api.rst>

Contact information
===================

.. only:: html

    :Release: |release|
    :Date: |today|

:Website: `http://commonroad.in.tum.de <https://commonroad.in.tum.de/>`_
:Email: `commonroad@lists.lrz.de <commonroad@lists.lrz.de>`_


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
