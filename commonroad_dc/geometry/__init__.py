import warnings

# Issue a deprecation warning when the package is imported
warnings.warn(
    "\nStarting from version 2025.1. the 'commonroad_dc.geometry' submodule has been moved to a standalone package. \n"
    "Please install the standalone package 'pip install commonroad-clcs' and update your imports to 'from commonroad_clcs import ...'. \n"
    "If you wish to use the old version, please run 'pip install commonroad-drivability-checker==2024.1'. \n",
    DeprecationWarning,
    stacklevel=2
)
