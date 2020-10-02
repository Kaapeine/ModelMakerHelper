#!/usr/bin/env python

import sys

from urdf_tweaker.interface import BasicInterface
from rqt_gui.main import Main

plugin = 'urdf_tweaker'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))