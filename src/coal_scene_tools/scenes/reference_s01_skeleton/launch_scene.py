#!/usr/bin/env python3
import subprocess
if __name__ == '__main__':
 raise SystemExit(subprocess.call(['ros2','run','coal_scene_tools','scene_loader','--scene-id','reference_s01_skeleton','--allow-skeleton']))
