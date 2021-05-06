# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys

print('documentation root' + os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../commonroad_dc')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../tutorials')))
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__) + '/../../third_party/commonroad-vehicle-models/Python'))
#sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../commonroad-io')) # ToDo: change this
import commonroad_dc.pycrcc as pycrcc


print("building documentation for the library {}".format(pycrcc.__file__))

# -- Project information -----------------------------------------------------

project = 'CommonRoad Drivability Checker'
copyright = '2021, Technical University of Munich, Professorship Cyber-Physical Systems'
author = 'Technical University of Munich, Professorship Cyber-Physical Systems'

# The full version, including alpha/beta/rc tags
release = '2021.1'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [ 
    'breathe', 
    'nbsphinx',
    'nbsphinx_link',
    'sphinx.ext.autodoc',
    'IPython.sphinxext.ipython_directive',
    'IPython.sphinxext.ipython_console_highlighting',
    'sphinx.ext.inheritance_diagram',
    'sphinx.ext.doctest',
    'sphinx.ext.coverage',
    'sphinx.ext.mathjax',
    'sphinx.ext.viewcode'
]

nbsphinx_execute = 'never'

# Add any paths that contain templates here, relative to this directory.
#templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store', '**.ipynb_checkpoints']


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'

html_theme_options = {
    'canonical_url': '',
    'analytics_id': '',
    'logo_only': False,
    'display_version': True,
    'prev_next_buttons_location': 'bottom',
    'style_external_links': False,
    # Toc options
    'collapse_navigation': False,
    'sticky_navigation': True,
    'navigation_depth': 4,
    'includehidden': True,
    'titles_only': False
}

html_logo = '_img/commonroad_white150.png'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
#html_static_path = ['_static']

# Breathe Configuration
breathe_default_project = 'crcc'

add_module_names = False
