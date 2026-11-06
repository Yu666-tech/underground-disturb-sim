#!/usr/bin/env python3
import runpy
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
tools=Path(get_package_share_directory('coal_scene_tools'))/'tools'
sys.path.insert(0,str(tools))
runpy.run_path(str(tools/'validate_scene.py'),run_name='__main__')
