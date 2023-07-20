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
from pathlib import Path
import shutil

# set required paths
current_file_dir = Path(__file__).parent
root_dir = current_file_dir.parent
build_dir = list(root_dir.glob("./build/temp.*"))[0]
build_python_bindings = list(build_dir.joinpath('build/cpp').resolve().glob('./*.so'))

print('documentation root {}'.format(os.path.abspath(root_dir)))

# NOTE:
# The python binding libraries (pycrcc.*.so and pycrcc.*.so) need to be copied to the source directory such that Sphinx
# can properly import them for building the documentation
# If building the documentation is deactivated, this step is done in setup.py
for file in build_python_bindings:
    if not file.exists():
        raise FileNotFoundError("Sphinx Error: Library pycrcc can not be found at {}". format(os.path.abspath(file)))
    shutil.copy(str(file), os.path.join(str(root_dir.resolve()), 'commonroad_dc'))

# Add source root and tutorial folder to sys.path
sys.path.insert(0, str((root_dir / 'tutorials').resolve()))
sys.path.insert(0, str(root_dir))

# If you get an import error in the following line: See note above
from commonroad_dc.__version__ import __version__
try:
    import commonroad_dc.pycrcc as pycrcc
except ImportError:
    raise ImportError("Sphinx Error: Library pycrcc is required to build the documentation and can not be found")

try:
    import commonroad_dc.pycrccosy as pycrccosy
except ImportError:
    raise ImportError("Sphinx Error: Library pycrccosy is required to build the documentation and can not be found")

print("building documentation for the library {}".format(pycrcc.__file__))
print("building documentation for the library {}".format(pycrccosy.__file__))

# -- Project information -----------------------------------------------------

project = 'CommonRoad Drivability Checker'
copyright = '2023, Technical University of Munich, Professorship Cyber-Physical Systems'
author = 'Technical University of Munich, Professorship Cyber-Physical Systems'

# The full version, including alpha/beta/rc tags
release = __version__


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
