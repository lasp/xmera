#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import subprocess
import sys
from pathlib import Path
import pydata_sphinx_theme

# -- Path setup ----------------------------------------------------------------

_examples_root = Path(os.path.join('..', '..', 'examples')).resolve()
_project_root = Path(os.path.join('..', '..')).resolve()

# Remove dist/ from sys.path so that the real (partial) xmera namespace package
# doesn't shadow autodoc_mock_imports during doc builds.
# The editable install (.pth file) adds dist/ automatically.
_dist_dir = str((_project_root / 'dist').resolve())
sys.path[:] = [p for p in sys.path if not os.path.realpath(p).startswith(_dist_dir)]

# Also remove any cached xmera modules so mocks take effect
for _key in list(sys.modules.keys()):
    if _key == 'xmera' or _key.startswith('xmera.'):
        del sys.modules[_key]

sys.path.insert(0, str(Path(os.path.join('..', '..', 'dist', 'xmera')).resolve()))
sys.path.insert(0, str(_examples_root))

# Add BskSim and sub-scenario directories so cross-module imports resolve
for _subdir in [
    'BskSim',
    os.path.join('BskSim', 'models'),
    os.path.join('BskSim', 'scenarios'),
    os.path.join('BskSim', 'plotting'),
    'MultiSatBskSim',
    os.path.join('MultiSatBskSim', 'modelsMultiSat'),
    os.path.join('MultiSatBskSim', 'scenariosMultiSat'),
    os.path.join('MultiSatBskSim', 'plottingMultiSat'),
    'OpNavScenarios',
    os.path.join('OpNavScenarios', 'modelsOpNav'),
    os.path.join('OpNavScenarios', 'scenariosOpNav'),
    os.path.join('OpNavScenarios', 'plottingOpNav'),
    os.path.join('OpNavScenarios', 'scenariosOpNav', 'OpNavMC'),
    os.path.join('OpNavScenarios', 'scenariosOpNav', 'CNN_ImageGen'),
    'MonteCarloExamples',
    'Support',
]:
    _path = _examples_root / _subdir
    if _path.is_dir():
        sys.path.insert(0, str(_path))

print("================")
print(sys.path)
print("================")

# -- Autodoc mock imports ------------------------------------------------------
# Dynamically discover all xmera.* imports from example scripts so autodoc
# doesn't fail when the C++ package isn't installed.

_xmera_mocks = {'xmera'}
for _py_file in _examples_root.rglob('*.py'):
    for _line in _py_file.read_text(errors='ignore').splitlines():
        _m = re.match(r'from\s+(xmera\S*)\s+import|import\s+(xmera\S*)', _line)
        if _m:
            _mod = _m.group(1) or _m.group(2)
            # Add all parent modules too (xmera, xmera.simulation, etc.)
            _parts = _mod.split('.')
            for _i in range(1, len(_parts) + 1):
                _xmera_mocks.add('.'.join(_parts[:_i]))

# BSK model modules (BSK_masters, BSK_Dynamics, etc.) are real Python files
# found via the sys.path entries above — they don't need mocking.
# Only xmera.* (C++ bindings) needs mocking.

autodoc_mock_imports = sorted(_xmera_mocks) + ['pytest']

# -- General configuration -----------------------------------------------------

html_theme = "pydata_sphinx_theme"
html_theme_options = {
  "navigation_depth": 4,
  "show_nav_level": 1,
  "icon_links": [
    {
      "name": "GitHub",
      "url": "https://github.com/lasp/xmera",
      "icon": "fa-brands fa-square-github",
      "type": "fontawesome",
    }
  ]
}

extensions = ['breathe',
              'pydata_sphinx_theme',
              'sphinx_copybutton',
              'sphinx_design',
              'sphinx.ext.autodoc',
              'sphinx.ext.mathjax']
breathe_projects = { "xmera": "../doxygen_output_xml" }
breathe_default_project = "xmera"
templates_path = ['_templates']
html_static_path = ['_static']
source_suffix = {'.rst': 'restructuredtext'}
master_doc = 'index'
project = 'xmera'
copyright = '2024, University of Colorado'

exclude_patterns = []
# include_patterns = ['Xmera/**']
highlight_language = 'c++'
pygments_style = 'sphinx'
todo_include_todos = False
htmlhelp_basename = 'xmeradoc'

# Custom CSS to hide sidebar children for large payload/base-class sections
html_css_files = ['custom.css']

# Suppress warnings about mocked objects (BSK modules aren't installed at doc-build time)
suppress_warnings = ['autodoc.mocked_object']

# ---------------------------------------------------------------------------
# Fix Breathe ndash/mdash rendering bug
# Breathe injects literal "&#8212;" text for <ndash/> and <mdash/> XML elements
# as a placeholder for block-quote attribution, but the placeholder leaks into
# normal paragraphs.  This doctree-resolved handler replaces them with real
# Unicode dashes.
# ---------------------------------------------------------------------------
from docutils import nodes as _nodes

def _fix_breathe_dashes(app, doctree, docname):
    for node in doctree.traverse(_nodes.Text):
        text = str(node)
        if '&#8212;' in text or '&#8211;' in text:
            new_text = text.replace('&#8212;', '\u2014').replace('&#8211;', '\u2013')
            node.parent.replace(node, _nodes.Text(new_text))

def setup(app):
    app.connect('doctree-resolved', _fix_breathe_dashes)
