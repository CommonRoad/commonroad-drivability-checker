import logging

try:
    import crmonitor
except ModuleNotFoundError:
    logging.warning("Traffic Rule Partial Cost Functions are not available. Please install the crmonitor package. \n"
                    "https://gitlab.lrz.de/cps/commonroad/commonroad-stl-monitor/")