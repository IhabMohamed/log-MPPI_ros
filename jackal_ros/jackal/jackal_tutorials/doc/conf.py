# -*- coding: utf-8 -*-

import os
import sys
import xml.etree.ElementTree as etree

sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.extlinks',
    'sphinx.ext.doctest',
    'sphinx.ext.viewcode',
]

extlinks = {
    'roswiki': ('http://wiki.ros.org/%s', ''),
}

source_suffix = '.rst'
master_doc = 'index'

project = u'jackal_tutorials'
copyright = u'2015, Clearpath Robotics'

# Get version number from package.xml.
tree = etree.parse('../package.xml')
version = tree.find("version").text
release = version

html_theme = 'nature'
htmlhelp_basename = 'jackal_tutorialsdoc'
templates_path = ['./templates']
html_static_path = ['./static']

html_sidebars = {
   '**': ['sidebartoc.html', 'sourcelink.html', 'searchbox.html']
}

rst_prolog = """
.. |ros_distro| replace:: indigo
.. |ubuntu_distro| replace:: trusty
"""
#.. ubuntu_distro: trusty

