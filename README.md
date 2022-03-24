CommonRoad Drivability Checker
------------------------------

Collision avoidance, kinematic feasibility, and road-compliance must be
validated to ensure the drivability of planned motions for autonomous
vehicles. The CommonRoad Drivability Checker toolbox unifies these checks
in order to simplify the development and validation of motion planning
algorithms. It is compatible with the CommonRoad benchmark suite, which
additionally facilitates and drastically reduces the effort of the development
of motion planning algorithms.

Please post questions, bug reports, etc. related to our tools or website in our [forum](https://commonroad.in.tum.de/forum/).

Installation
------------
To install the Python package of the drivability checker, please run:
```
pip install commonroad-drivability-checker
```

Documentation
-------------
A full documentation as well as tutorials to get started with the tool can be found on our [toolpage](https://commonroad.in.tum.de/drivability-checker).


Publication
-----------
#### CommonRoad Drivability Checker: Simplifying the Development and Validation of Motion Planning Algorithms
Christian Pek, Vitaliy Rusinov, Stefanie Manzinger, Murat Can Üste, and Matthias Althoff

Abstract— Collision avoidance, kinematic feasibility, and road-compliance must be validated to ensure the drivability
of planned motions for autonomous vehicles. Although these tasks are highly repetitive, computationally efficient
toolboxes are still unavailable. The CommonRoad Drivability Checker—an open-source toolbox—unifies these mentioned
checks. It is compatible with the CommonRoad benchmark suite, which additionally facilitates the development of motion
planners. Our toolbox drastically reduces the effort of developing and validating motion planning algorithms. Numerical
experiments show that our toolbox is real-time capable and can be used in real test vehicles.

Fulltext is available on [mediaTUM](https://mediatum.ub.tum.de/doc/1546126/).

##### Bibtex:
```
@inproceedings{ PekIV20.pdf,
	author = "Christian Pek, Vitaliy Rusinov, Stefanie Manzinger, Murat Can Üste, and Matthias Althoff",
	title = "CommonRoad Drivability Checker: Simplifying the Development and Validation of Motion Planning Algorithms",
	pages = "1-8",
	booktitle = "Proc. of the IEEE Intelligent Vehicles Symposium",
	year = "2020",
	abstract = "Collision avoidance, kinematic feasibility, and road-compliance must be validated to ensure the drivability of planned motions for autonomous vehicles. Although these tasks are highly repetitive, computationally efficient toolboxes are still unavailable. The CommonRoad Drivability Checker— an open-source toolbox—unifies these mentioned checks. It is compatible with the CommonRoad benchmark suite, which additionally facilitates the development of motion planners. Our toolbox drastically reduces the effort of developing and validating motion planning algorithms. Numerical experiments show that our toolbox is real-time capable and can be used in real test vehicles."
}
```


Licensing note
--------------
Please note, that the drivability checker contains a dependency to the third party library [Triangle](https://www.cs.cmu.edu/~quake/triangle.html)
which states:
```
Please note that although Triangle is freely available, it is copyrighted by the author and may 
not be sold or included in commercial products without a license.
```
Please be aware of that when using the drivability checker in your projects. See the original [licensing conditions](https://github.com/wo80/Triangle/blob/master/src/Triangle/README) 
of the Triangle library as well as the [Triangle website](https://www.cs.cmu.edu/~quake/triangle.html) for details.