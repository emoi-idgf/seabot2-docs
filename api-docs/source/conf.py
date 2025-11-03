# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Seabot2 ROS2 API'
copyright = '2025, Seabot2 Team, ENSTA'
author = 'Seabot2 Team, ENSTA'
release = '0.0.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

enable_breathe = True
enable_exhale = True

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    ]

if enable_breathe:
    extensions.append('breathe')

if enable_exhale:
    extensions.append('exhale')

templates_path = ['_templates']
exclude_patterns = []


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_theme_options = {
    'style_nav_header_background': '#1D8BC2'
}
html_logo = '../../user-manual-docs/source/_static/logo/seabot2_logo.png'

html_static_path = ['_static']


# -- Breathe configuration -------------------------------------------------
breathe_projects = {
    "seabot2": "./_doxygen/xml"
}
breathe_default_project = "seabot2"

# -- Exhale configuration -------------------------------------------------
exhale_args = {
    # These arguments are required
    "containmentFolder":     "./api",
    "rootFileName":          "library_root.rst",
    "doxygenStripFromPath":  "..",
    # Heavily encouraged optional argument (see docs)
    "rootFileTitle":         "Seabot2 C++ API",
    # Suggested optional arguments
    "createTreeView":        True,
    # TIP: if using the sphinx-bootstrap-theme, you need
    # "treeViewIsBootstrap": True,
    "exhaleExecutesDoxygen": False,
    "exhaleDoxygenStdin":    "INPUT = ../../../seabot2-ros/src/"
}

# Tell sphinx what the primary language being documented is.
primary_domain = 'cpp'

# Tell sphinx what the pygments highlight language should be.
highlight_language = 'cpp'
