# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Seabot2 User Manual'
copyright = '2025, Seabot2 Team, ENSTA'
author = 'Seabot2 Team, ENSTA'
release = '0.0.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

enable_breathe = False

extensions = [
    'sphinx_copybutton',
    ]

if enable_breathe:
    extensions.append('breathe')

templates_path = ['_templates']
exclude_patterns = []

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_theme_options = {
    'style_nav_header_background': '#1D8BC2'
}
html_logo = '_static/logo/seabot2_logo.png'

html_static_path = ['_static']


# -- Breathe configuration -------------------------------------------------
breathe_projects = {
    "seabot2": "./_doxygen/xml"
}
breathe_default_project = "seabot2"