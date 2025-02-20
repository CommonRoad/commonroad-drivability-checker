"""
NOTE: This module has been deprecated after version 2025.1 of commonroad-drivability-checker.
      The curvilinear coordinate system is now installable via: pip install commonroad-clcs
"""

import warnings

# Intercept access to non-existent methods
def __getattr__(name):
    # Issue a deprecation warning for non-existent methods
    warnings.warn(
        "\nStarting from version 2025.1. the 'commonroad_dc.geometry' submodule has been moved to a standalone package. \n"
        "Please install the standalone package 'pip install commonroad-clcs' and update your imports to 'from commonroad_clcs import ...'. \n"
        "If you wish to use the old version, please run 'pip install commonroad-drivability-checker==2024.1'. \n",
        DeprecationWarning,
        stacklevel=2
    )
    # Raise an AttributeError to mimic Python's default behavior
    raise ImportError(f"\nModule commonroad_dc.geometry.geometry has been moved to the commonroad-clcs package. \n"
                      f"Please install the curvilinear coordinate system package via 'pip install commonroad-clcs'")