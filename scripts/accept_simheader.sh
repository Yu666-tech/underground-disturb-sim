#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
mkdir -p evidence
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
python3 src/coal_scene_tools/tests/test_validator.py
ros2 run coal_scene_tools scene_validator.py src/coal_scene_tools/scenes/reference_s01_skeleton
ros2 run coal_scene_tools scene_loader --scene-id reference_s01_skeleton --allow-skeleton
python3 src/coal_platform_devtools/scripts/check_fixtures.py --launch --output evidence/simheader_fixtures.json
python3 scripts/check_return_chain.py
python3 scripts/verify_install.py --launch --workspace "$PWD" --output evidence/simheader_gazebo_verification.json
