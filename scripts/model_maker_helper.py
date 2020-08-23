#!/usr/bin/env python

import sys

from model_maker_helper.interface import BasicInterface
from rqt_gui.main import Main

plugin = 'model_maker_helper'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))